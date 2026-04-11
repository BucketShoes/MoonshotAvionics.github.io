#ifndef LOG_STORE_H
#define LOG_STORE_H

#include <esp_partition.h>
#include <Preferences.h>
#include <string.h>

#define LOG_HDR_SIZE       6
#define LOG_MAX_PAYLOAD    254
#define LOG_ERASED_MARKER  0xFF
#define SECTOR_SIZE        4096
#define CHUNK_SIZE         256
#define INDEX_ENTRY_SIZE   4
#define INDEX_ERASED       0xFFFFFFFF

// NVS keys (stored within the namespace provided by caller)
// "log_rec"  = uint32_t recordCounter at last chunk boundary
// "log_vpos" = uint32_t virtualPos at last chunk boundary
// These are written every 256 records (when a new index entry is created).
// On boot, we read these and scan forward within the last chunk to find exact position.

class LogStore {
public:
  bool begin(const char* dataPartLabel, const char* indexPartLabel,
             const char* nvsNamespace = "logstore") {
    Serial.println("LogStore::begin()");
    writesSinceBoot_ = 0;

    // Open NVS namespace for log state persistence
    nvsOk_ = nvs_.begin(nvsNamespace, false);
    if (!nvsOk_) Serial.println("  NVS open failed");

    // Find partitions by label
    esp_partition_iterator_t it = esp_partition_find(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it) {
      const esp_partition_t* p = esp_partition_get(it);
      if (strcmp(p->label, dataPartLabel) == 0) dataPart_ = p;
      if (strcmp(p->label, indexPartLabel) == 0) indexPart_ = p;
      it = esp_partition_next(it);
    }
    if (!dataPart_ || !indexPart_) {
      Serial.println("  PARTITIONS NOT FOUND");
      return false;
    }
    ringSize_ = dataPart_->size;
    maxIndexEntries_ = indexPart_->size / INDEX_ENTRY_SIZE;
    Serial.print("  DATA @0x"); Serial.print(dataPart_->address, HEX);
    Serial.print(" sz="); Serial.println(dataPart_->size);
    Serial.print("  IDX  @0x"); Serial.print(indexPart_->address, HEX);
    Serial.print(" sz="); Serial.println(indexPart_->size);

    recoverState();
    return true;
  }

  void eraseLogs() {
    if (!dataPart_ || !indexPart_) return;
    Serial.print("LogStore eraseLogs...");
    esp_partition_erase_range(indexPart_, 0, indexPart_->size);
    for (uint32_t off = 0; off < dataPart_->size; off += 65536) {
      uint32_t sz = 65536;
      if (off + sz > dataPart_->size) sz = dataPart_->size - off;
      esp_partition_erase_range(dataPart_, off, sz);
      if ((off % (256*1024)) == 0) { Serial.print("."); vTaskDelay(1); }
    }
    virtualPos_ = 0;
    recordCounter_ = 0;
    currentSector_ = 0;
    protectionVpos_ = 0;
    logFull_ = false;
    // Clear only log-related NVS keys (not other config in this namespace)
    if (nvsOk_) {
      nvs_.remove("log_rec");
      nvs_.remove("log_vpos");
      nvs_.remove("log_prot");
    }
    Serial.println(" done");
  }

  bool isReady() const { return dataPart_ != nullptr && indexPart_ != nullptr; }

  // Set log protection point at the current write position.
  // The ring buffer will refuse to overwrite data at or after this point.
  // Call when arming. Persisted to NVS so it survives power cycles.
  // Returns the record counter at the protection point.
  uint32_t setProtectionPoint() {
    protectionVpos_ = virtualPos_;
    if (nvsOk_) nvs_.putUInt("log_prot", protectionVpos_);
    Serial.print("LogStore: protection set vpos=0x");
    Serial.println(protectionVpos_, HEX);
    return recordCounter_;
  }

  // Clear protection point (call on disarm, before re-arm moves it forward).
  void clearProtectionPoint() {
    protectionVpos_ = 0;
    logFull_ = false;  // allow writes again now that protection is gone
    if (nvsOk_) nvs_.remove("log_prot");
    Serial.println("LogStore: protection cleared");
  }

  bool isProtected() const { return protectionVpos_ != 0; }
  bool isFull() const { return logFull_; }

  int32_t writeRecord(const uint8_t* payload, uint8_t payloadLen,
                      int8_t snr, uint32_t timestampMs) {
    if (!isReady()) return -1;
    if (payloadLen > LOG_MAX_PAYLOAD || payloadLen == 0) return -1;
    if (virtualPos_ >= 0xFFFF0000UL || recordCounter_ >= 0xFFFFFFF0UL) return -1;
    if (logFull_) return -1;

    bool dbg = (writesSinceBoot_ < 3);
    uint16_t totalSize = LOG_HDR_SIZE + payloadLen;

    // Check if record fits in current sector
    uint32_t soff = virtualPos_ % SECTOR_SIZE;
    uint32_t nextVpos = virtualPos_;
    if (soff + totalSize > SECTOR_SIZE) {
      // Would need to advance to next sector
      nextVpos += (SECTOR_SIZE - soff);
    }

    // Protection check: if protected, the ring must not overwrite data at or
    // after the protection point. We check if the erase-ahead sector (the sector
    // after where we're about to write) would destroy data in the protected range.
    // The protected range is: protectionVpos_ to (protectionVpos_ + ringSize_).
    // We're safe as long as the gap between our write position and the protection
    // point hasn't closed to less than ringSize_ bytes.
    if (protectionVpos_ != 0) {
      // How far ahead of the protection point are we?
      // Both are virtual (monotonically increasing), so simple subtraction works.
      uint32_t aheadVpos = nextVpos + totalSize + SECTOR_SIZE; // worst case with erase-ahead
      if (aheadVpos - protectionVpos_ >= ringSize_) {
        logFull_ = true;
        Serial.println("LogStore: FULL (protection point reached)");
        return -1;
      }
    }

    // Now actually advance if needed
    if (soff + totalSize > SECTOR_SIZE) {
      virtualPos_ += (SECTOR_SIZE - soff);

      uint32_t newSector = physSector(virtualPos_);
      uint32_t aheadSector = physSector(virtualPos_ + SECTOR_SIZE);
      if (dbg) {
        Serial.print("  sector advance -> 0x"); Serial.print(newSector, HEX);
        Serial.print(" erase-ahead 0x"); Serial.println(aheadSector, HEX);
      }
      esp_partition_erase_range(dataPart_, aheadSector, SECTOR_SIZE);
      currentSector_ = newSector;
    }

    uint32_t physAddr = virtualPos_ % ringSize_;

    // Index entry at chunk boundary
    if ((recordCounter_ % CHUNK_SIZE) == 0) {
      uint32_t slot = (recordCounter_ / CHUNK_SIZE) % maxIndexEntries_;
      writeIndexEntry(slot, virtualPos_);
      // Persist counters to NVS at each chunk boundary
      saveStateToNvs();
      if (dbg) {
        Serial.print("  IDX["); Serial.print(slot);
        Serial.print("]=0x"); Serial.println(virtualPos_, HEX);
      }
    }

    // Build and write record
    uint8_t hdr[LOG_HDR_SIZE];
    hdr[0] = payloadLen;
    hdr[1] = (uint8_t)snr;
    hdr[2] = timestampMs & 0xFF;
    hdr[3] = (timestampMs >> 8) & 0xFF;
    hdr[4] = (timestampMs >> 16) & 0xFF;
    hdr[5] = (timestampMs >> 24) & 0xFF;

    esp_err_t e1 = esp_partition_write(dataPart_, physAddr, hdr, LOG_HDR_SIZE);
    esp_err_t e2 = esp_partition_write(dataPart_, physAddr + LOG_HDR_SIZE, payload, payloadLen);

    if (dbg || e1 != ESP_OK || e2 != ESP_OK) {
      uint8_t vfy[4];
      esp_partition_read(dataPart_, physAddr, vfy, 4);
      Serial.print("  W#"); Serial.print(recordCounter_);
      Serial.print(" @0x"); Serial.print(physAddr, HEX);
      Serial.print(" e="); Serial.print(e1); Serial.print(","); Serial.print(e2);
      Serial.print(" vfy=");
      for(int i=0;i<4;i++){if(vfy[i]<0x10)Serial.print("0");Serial.print(vfy[i],HEX);}
      Serial.println();
    }

    uint32_t recNum = recordCounter_;
    virtualPos_ += totalSize;
    recordCounter_++;
    writesSinceBoot_++;
    return (int32_t)recNum;
  }

  int readRecord(uint32_t recordNum, uint8_t* payloadBuf, size_t bufSize,
                 int8_t* snrOut, uint32_t* timestampOut) {
    if (!isReady()) return -1;
    if (recordNum >= recordCounter_) return -1;
    uint32_t slot = (recordNum / CHUNK_SIZE) % maxIndexEntries_;
    uint32_t chunkVP = readIndexEntry(slot);
    if (chunkVP == INDEX_ERASED) return -1;
    if (virtualPos_ > ringSize_ && chunkVP < (virtualPos_ - ringSize_)) return -1;

    uint32_t scanPos = chunkVP;
    uint32_t target = recordNum % CHUNK_SIZE;
    for (uint32_t i = 0; i < target; i++) {
      uint8_t len = readByte(scanPos);
      if (len == LOG_ERASED_MARKER || len == 0 || len > LOG_MAX_PAYLOAD) return -1;
      scanPos += LOG_HDR_SIZE + len;
      scanPos = skipGap(scanPos);
    }

    uint8_t hdr[LOG_HDR_SIZE];
    readBytes(scanPos, hdr, LOG_HDR_SIZE);
    uint8_t pLen = hdr[0];
    if (pLen == LOG_ERASED_MARKER || pLen == 0 || pLen > LOG_MAX_PAYLOAD || pLen > bufSize)
      return -1;
    *snrOut = (int8_t)hdr[1];
    *timestampOut = hdr[2] | ((uint32_t)hdr[3]<<8) | ((uint32_t)hdr[4]<<16) | ((uint32_t)hdr[5]<<24);
    readBytes(scanPos + LOG_HDR_SIZE, payloadBuf, pLen);
    return pLen;
  }

  uint32_t getRecordCounter() const { return recordCounter_; }
  uint32_t getVirtualPos() const { return virtualPos_; }
  uint32_t getRingSize() const { return ringSize_; }

  // Read a record as its raw flash bytes: [length(1)][snr(1)][timestamp(4)][payload(length)].
  // Returns total bytes written to buf (LOG_HDR_SIZE + payloadLen), or -1 on error.
  // This is used for log download chunks which send records in their flash format.
  int readRecordRaw(uint32_t recordNum, uint8_t* buf, size_t bufSize) {
    if (!isReady()) return -1;
    if (recordNum >= recordCounter_) return -1;
    uint32_t slot = (recordNum / CHUNK_SIZE) % maxIndexEntries_;
    uint32_t chunkVP = readIndexEntry(slot);
    if (chunkVP == INDEX_ERASED) return -1;
    if (virtualPos_ > ringSize_ && chunkVP < (virtualPos_ - ringSize_)) return -1;

    uint32_t scanPos = chunkVP;
    uint32_t target = recordNum % CHUNK_SIZE;
    for (uint32_t i = 0; i < target; i++) {
      uint8_t len = readByte(scanPos);
      if (len == LOG_ERASED_MARKER || len == 0 || len > LOG_MAX_PAYLOAD) return -1;
      scanPos += LOG_HDR_SIZE + len;
      scanPos = skipGap(scanPos);
    }

    uint8_t hdr[LOG_HDR_SIZE];
    readBytes(scanPos, hdr, LOG_HDR_SIZE);
    uint8_t pLen = hdr[0];
    if (pLen == LOG_ERASED_MARKER || pLen == 0 || pLen > LOG_MAX_PAYLOAD) return -1;
    uint16_t totalSize = LOG_HDR_SIZE + pLen;
    if (totalSize > bufSize) return -1;

    memcpy(buf, hdr, LOG_HDR_SIZE);
    readBytes(scanPos + LOG_HDR_SIZE, buf + LOG_HDR_SIZE, pLen);
    return (int)totalSize;
  }

  struct SeqReader {
    LogStore* store;
    uint32_t  recIdx;
    uint32_t  endRec;
    uint32_t  scanPos;
    bool      valid;

    bool hasMore() const { return valid && recIdx < endRec; }
    uint32_t currentRec() const { return recIdx; }

    int readNext(uint8_t* payloadBuf, size_t bufSize, int8_t* snrOut, uint32_t* tsOut) {
      if (!valid || recIdx >= endRec) return -1;
      uint8_t hdr[LOG_HDR_SIZE];
      store->readBytes(scanPos, hdr, LOG_HDR_SIZE);
      uint8_t pLen = hdr[0];
      if (pLen == LOG_ERASED_MARKER || pLen == 0 || pLen > LOG_MAX_PAYLOAD) { valid = false; return -1; }
      if (pLen > bufSize) { valid = false; return -1; }
      *snrOut = (int8_t)hdr[1];
      *tsOut  = hdr[2] | ((uint32_t)hdr[3]<<8) | ((uint32_t)hdr[4]<<16) | ((uint32_t)hdr[5]<<24);
      store->readBytes(scanPos + LOG_HDR_SIZE, payloadBuf, pLen);
      scanPos += LOG_HDR_SIZE + pLen;
      scanPos  = store->skipGap(scanPos);
      recIdx++;
      return pLen;
    }
  };

  SeqReader seqReader(uint32_t startRec, uint32_t endRec = 0) {
    SeqReader r;
    r.store = this; r.recIdx = startRec;
    r.endRec = (endRec == 0) ? recordCounter_ : endRec;
    r.valid = false;
    if (!isReady() || startRec >= recordCounter_) return r;
    uint32_t slot = (startRec / CHUNK_SIZE) % maxIndexEntries_;
    uint32_t chunkVP = readIndexEntry(slot);
    if (chunkVP == INDEX_ERASED) return r;
    if (virtualPos_ > ringSize_ && chunkVP < (virtualPos_ - ringSize_)) return r;
    uint32_t scanPos = chunkVP;
    uint32_t target = startRec % CHUNK_SIZE;
    for (uint32_t i = 0; i < target; i++) {
      uint8_t len = readByte(scanPos);
      if (len == LOG_ERASED_MARKER || len == 0 || len > LOG_MAX_PAYLOAD) return r;
      scanPos += LOG_HDR_SIZE + len;
      scanPos = skipGap(scanPos);
    }
    r.scanPos = scanPos; r.valid = true;
    return r;
  }

  uint32_t getOldestRecord() const {
    if (!isReady() || recordCounter_ == 0) return 0;
    if (virtualPos_ <= ringSize_) return 0;
    // Data older than ringSize_ bytes ago has been overwritten.
    // Find the first index entry whose virtual offset is still within range.
    uint32_t oldVP = virtualPos_ - ringSize_;
    // Start searching from the slot after the current write slot, wrapping around.
    // This finds the oldest surviving chunk.
    uint32_t currentSlot = (recordCounter_ / CHUNK_SIZE) % maxIndexEntries_;
    for (uint32_t i = 0; i < maxIndexEntries_; i++) {
      uint32_t s = (currentSlot + 1 + i) % maxIndexEntries_;
      uint32_t v = readIndexEntry(s);
      if (v == INDEX_ERASED) continue;
      if (v >= oldVP) return s * CHUNK_SIZE;  // approximate — chunk granularity
    }
    return recordCounter_;  // nothing readable
  }

private:
  friend struct SeqReader;

  const esp_partition_t* dataPart_ = nullptr;
  const esp_partition_t* indexPart_ = nullptr;
  Preferences nvs_;
  bool nvsOk_ = false;
  uint32_t ringSize_ = 0;
  uint32_t maxIndexEntries_ = 0;
  uint32_t virtualPos_ = 0;
  uint32_t recordCounter_ = 0;
  uint32_t currentSector_ = 0;
  uint32_t writesSinceBoot_ = 0;
  uint32_t protectionVpos_ = 0;  // 0 = no protection; >0 = don't overwrite past here
  bool logFull_ = false;         // set when protection point blocks further writes

  uint32_t physSector(uint32_t vAddr) {
    return ((vAddr % ringSize_) / SECTOR_SIZE) * SECTOR_SIZE;
  }

  uint8_t readByte(uint32_t vA) {
    uint8_t b; esp_partition_read(dataPart_, vA % ringSize_, &b, 1); return b;
  }
  void readBytes(uint32_t vA, uint8_t* buf, size_t len) {
    uint32_t p = vA % ringSize_;
    if (p + len <= ringSize_) {
      esp_partition_read(dataPart_, p, buf, len);
    } else {
      size_t f = ringSize_ - p;
      esp_partition_read(dataPart_, p, buf, f);
      esp_partition_read(dataPart_, 0, buf + f, len - f);
    }
  }
  uint32_t readIndexEntry(uint32_t slot) const {
    if (slot >= maxIndexEntries_) return INDEX_ERASED;
    uint32_t v;
    esp_partition_read(indexPart_, slot * INDEX_ENTRY_SIZE, &v, 4);
    return v;
  }
  void writeIndexEntry(uint32_t slot, uint32_t vOff) {
    if (slot >= maxIndexEntries_) return;
    uint32_t off = slot * INDEX_ENTRY_SIZE;
    uint32_t secAddr = (off / SECTOR_SIZE) * SECTOR_SIZE;
    uint32_t existing = readIndexEntry(slot);
    if (existing != INDEX_ERASED) {
      esp_partition_erase_range(indexPart_, secAddr, SECTOR_SIZE);
    }
    esp_partition_write(indexPart_, off, &vOff, 4);
  }

  void saveStateToNvs() {
    if (!nvsOk_) return;
    nvs_.putUInt("log_rec", recordCounter_);
    nvs_.putUInt("log_vpos", virtualPos_);
  }

  uint32_t skipGap(uint32_t pos) {
    uint32_t s = pos % SECTOR_SIZE;
    if (s == 0) return pos;
    if (s < LOG_HDR_SIZE) return pos + (SECTOR_SIZE - s);
    uint8_t b = readByte(pos);
    if (b == LOG_ERASED_MARKER || b == 0) {
      uint32_t nx = pos + (SECTOR_SIZE - s);
      uint8_t nb = readByte(nx);
      if (nb > 0 && nb <= LOG_MAX_PAYLOAD) return nx;
    }
    return pos;
  }

  void recoverState() {
    Serial.print("  recover: ");

    // Try NVS first — this gives us the exact counter at the last chunk boundary
    uint32_t nvsRec = 0;
    uint32_t nvsVpos = 0;
    bool nvsValid = false;
    if (nvsOk_) {
      // Check if both keys exist (getUInt returns default 0xFFFFFFFF if missing)
      nvsRec = nvs_.getUInt("log_rec", 0xFFFFFFFF);
      nvsVpos = nvs_.getUInt("log_vpos", 0xFFFFFFFF);
      if (nvsRec != 0xFFFFFFFF && nvsVpos != 0xFFFFFFFF) {
        nvsValid = true;
        Serial.print("NVS rec="); Serial.print(nvsRec);
        Serial.print(" vpos=0x"); Serial.print(nvsVpos, HEX);
      } else {
        Serial.print("NVS empty");
      }
      // Load protection point (0 = not set)
      protectionVpos_ = nvs_.getUInt("log_prot", 0);
      if (protectionVpos_ != 0) {
        Serial.print(" prot=0x"); Serial.print(protectionVpos_, HEX);
      }
    }

    if (nvsValid) {
      // NVS tells us the state at the last chunk boundary.
      // Verify the index entry matches what NVS says.
      uint32_t slot = (nvsRec / CHUNK_SIZE) % maxIndexEntries_;
      uint32_t idxVal = readIndexEntry(slot);

      if (idxVal == nvsVpos) {
        // Consistent — scan forward from this chunk to find exact position.
        recordCounter_ = nvsRec;
        virtualPos_ = nvsVpos;
        scanForwardFromChunk();
        return;
      } else {
        Serial.print(" IDX mismatch(0x"); Serial.print(idxVal, HEX); Serial.print(")");
        // Fall through to flash-scan recovery
      }
    }

    // Fallback: scan flash index to find state (handles first boot, NVS loss, etc)
    recoverFromFlash();
  }

  // Scan forward from recordCounter_/virtualPos_ (set to chunk boundary)
  // to find the exact write position within the last partial chunk.
  void scanForwardFromChunk() {
    uint32_t scanPos = virtualPos_;
    uint32_t recsFound = 0;
    for (uint32_t i = 0; i < CHUNK_SIZE; i++) {
      scanPos = skipGap(scanPos);
      uint8_t len = readByte(scanPos);
      if (len == LOG_ERASED_MARKER || len == 0 || len > LOG_MAX_PAYLOAD) {
        // Could be sector gap — check
        uint32_t s = scanPos % SECTOR_SIZE;
        if (s != 0 && (len == LOG_ERASED_MARKER || len == 0)) {
          uint32_t nx = scanPos + (SECTOR_SIZE - s);
          uint8_t nb = readByte(nx);
          if (nb > 0 && nb <= LOG_MAX_PAYLOAD) { scanPos = nx; continue; }
        }
        break;
      }
      scanPos += LOG_HDR_SIZE + len;
      recsFound++;
    }

    recordCounter_ += recsFound;
    virtualPos_ = scanPos;
    currentSector_ = physSector(virtualPos_);

    // Pre-clear the NEXT sector (so it's ready when we cross into it).
    // Skip if protection means we're already full — don't erase protected data.
    uint32_t aheadSector = physSector(virtualPos_ + SECTOR_SIZE);
    bool aheadSafe = true;
    if (protectionVpos_ != 0) {
      uint32_t aheadVpos = virtualPos_ + SECTOR_SIZE;
      if (aheadVpos - protectionVpos_ >= ringSize_) aheadSafe = false;
    }
    if (aheadSector != currentSector_ && aheadSafe) {
      esp_partition_erase_range(dataPart_, aheadSector, SECTOR_SIZE);
    }

    Serial.print(" scanned="); Serial.print(recsFound);
    Serial.print(" -> rec="); Serial.print(recordCounter_);
    Serial.print(" vpos=0x"); Serial.println(virtualPos_, HEX);

    // Check if already full (protection point active and ring has lapped it)
    if (protectionVpos_ != 0 && virtualPos_ - protectionVpos_ >= ringSize_) {
      logFull_ = true;
      Serial.println("  WARNING: log full on boot (protection point reached)");
    }
  }

  // Flash-only recovery: scan index entries to find last written chunk.
  // Used on first boot or if NVS is lost/corrupt.
  void recoverFromFlash() {
    Serial.print(" flash-scan: ");

    // Find first erased index slot
    uint32_t firstErased = maxIndexEntries_;
    for (uint32_t i = 0; i < maxIndexEntries_; i++) {
      if (readIndexEntry(i) == INDEX_ERASED) { firstErased = i; break; }
    }
    Serial.print("1stErased="); Serial.print(firstErased);

    if (firstErased == 0) {
      // Slot 0 is erased. Check if ALL slots are erased (fresh flash)
      uint32_t lastVal = readIndexEntry(maxIndexEntries_ - 1);
      if (lastVal == INDEX_ERASED) {
        virtualPos_ = 0;
        recordCounter_ = 0;
        currentSector_ = 0;
        esp_partition_erase_range(dataPart_, SECTOR_SIZE, SECTOR_SIZE);
        Serial.println(" -> empty, pre-cleared sector 1");
        return;
      }
      // Index ring has wrapped: slot 0 was overwritten then erased, but later slots have data
      firstErased = maxIndexEntries_;
      Serial.print(" (full)");
    }

    uint32_t lastSlot = firstErased - 1;

    // Check for index ring wrap: are there non-erased entries after the gap?
    uint32_t passes = 0;
    if (firstErased < maxIndexEntries_) {
      for (uint32_t i = firstErased + 1; i < maxIndexEntries_; i++) {
        if (readIndexEntry(i) != INDEX_ERASED) { passes = 1; break; }
      }
    }

    uint32_t recBase = (passes * maxIndexEntries_ + lastSlot) * CHUNK_SIZE;
    uint32_t chunkVP = readIndexEntry(lastSlot);
    Serial.print(" lastSlot="); Serial.print(lastSlot);
    Serial.print(" vp=0x"); Serial.print(chunkVP, HEX);
    Serial.print(" base="); Serial.print(recBase);

    recordCounter_ = recBase;
    virtualPos_ = chunkVP;
    scanForwardFromChunk();

    // Save recovered state to NVS so next boot is fast
    saveStateToNvs();
  }
};

#endif
