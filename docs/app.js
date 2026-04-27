(function() {
  "use strict";

  // =============================================================
  // BASE URL CONFIG
  // =============================================================
  // When the page is served from the base station WiFi AP, the URL is the AP IP.
  // When loaded from a static file, GitHub Pages, or any other origin, the user
  // can enter the base station IP manually. Default: 192.168.4.1
  // All HTTP requests (status, logs, commands) and WebSocket connections use this.

  function getBaseHost() {
    var el = document.getElementById('base-url');
    if (el && el.value) return el.value.trim();
    return '192.168.4.1';
  }

  // HTTP base: always use http:// to talk to the ESP32 AP (no TLS on the device)
  function getBaseHttp() {
    return 'http://' + getBaseHost();
  }

  // WebSocket base
  function getBaseWs() {
    return 'ws://' + getBaseHost() + ':80/ws';
  }

  // =============================================================
  // Pure JS SHA-256 (no crypto.subtle needed - works over HTTP)
  // =============================================================

  var SHA256_K = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
  ];

  function sha256(msg) {
    // msg is Uint8Array, returns Uint8Array(32)
    var len = msg.length;
    var bitLen = len * 8;

    // Padding
    var padLen = (len + 9);
    if (padLen % 64 !== 0) padLen += 64 - (padLen % 64);
    var padded = new Uint8Array(padLen);
    padded.set(msg);
    padded[len] = 0x80;
    // Length in bits as big-endian 64-bit at end
    var dv = new DataView(padded.buffer);
    dv.setUint32(padLen - 4, bitLen, false);

    var H = [0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19];
    var W = new Array(64);

    for (var block = 0; block < padLen; block += 64) {
      for (var i = 0; i < 16; i++) {
        W[i] = dv.getUint32(block + i * 4, false);
      }
      for (var i = 16; i < 64; i++) {
        var s0 = rr(W[i-15], 7) ^ rr(W[i-15], 18) ^ (W[i-15] >>> 3);
        var s1 = rr(W[i-2], 17) ^ rr(W[i-2], 19) ^ (W[i-2] >>> 10);
        W[i] = (W[i-16] + s0 + W[i-7] + s1) | 0;
      }

      var a=H[0],b=H[1],c=H[2],d=H[3],e=H[4],f=H[5],g=H[6],h=H[7];
      for (var i = 0; i < 64; i++) {
        var S1 = rr(e,6) ^ rr(e,11) ^ rr(e,25);
        var ch = (e & f) ^ (~e & g);
        var t1 = (h + S1 + ch + SHA256_K[i] + W[i]) | 0;
        var S0 = rr(a,2) ^ rr(a,13) ^ rr(a,22);
        var maj = (a & b) ^ (a & c) ^ (b & c);
        var t2 = (S0 + maj) | 0;
        h=g; g=f; f=e; e=(d+t1)|0; d=c; c=b; b=a; a=(t1+t2)|0;
      }
      H[0]=(H[0]+a)|0; H[1]=(H[1]+b)|0; H[2]=(H[2]+c)|0; H[3]=(H[3]+d)|0;
      H[4]=(H[4]+e)|0; H[5]=(H[5]+f)|0; H[6]=(H[6]+g)|0; H[7]=(H[7]+h)|0;
    }

    var out = new Uint8Array(32);
    var odv = new DataView(out.buffer);
    for (var i = 0; i < 8; i++) odv.setUint32(i * 4, H[i], false);
    return out;
  }

  function rr(n, b) { return ((n >>> b) | (n << (32 - b))) >>> 0; }

  function hmacSha256(key, msg) {
    // key and msg are Uint8Array, returns Uint8Array(32)
    // Key longer than 64 bytes gets hashed first
    if (key.length > 64) key = sha256(key);
    var ipad = new Uint8Array(64);
    var opad = new Uint8Array(64);
    for (var i = 0; i < 64; i++) {
      var k = i < key.length ? key[i] : 0;
      ipad[i] = k ^ 0x36;
      opad[i] = k ^ 0x5c;
    }
    // inner = SHA256(ipad + msg)
    var inner = new Uint8Array(64 + msg.length);
    inner.set(ipad);
    inner.set(msg, 64);
    var innerHash = sha256(inner);
    // outer = SHA256(opad + innerHash)
    var outer = new Uint8Array(64 + 32);
    outer.set(opad);
    outer.set(innerHash, 64);
    return sha256(outer);
  }

  function pbkdf2Sha256(password, salt, iterations, keyLen) {
    // password and salt are Uint8Array
    // Returns Uint8Array of keyLen bytes
    var numBlocks = Math.ceil(keyLen / 32);
    var result = new Uint8Array(numBlocks * 32);

    for (var block = 1; block <= numBlocks; block++) {
      // U1 = HMAC(password, salt + INT32_BE(block))
      var saltBlock = new Uint8Array(salt.length + 4);
      saltBlock.set(salt);
      saltBlock[salt.length] = (block >> 24) & 0xFF;
      saltBlock[salt.length + 1] = (block >> 16) & 0xFF;
      saltBlock[salt.length + 2] = (block >> 8) & 0xFF;
      saltBlock[salt.length + 3] = block & 0xFF;

      var u = hmacSha256(password, saltBlock);
      var xor = new Uint8Array(u);

      for (var i = 1; i < iterations; i++) {
        u = hmacSha256(password, u);
        for (var j = 0; j < 32; j++) xor[j] ^= u[j];
      }
      result.set(xor, (block - 1) * 32);
    }
    return result.slice(0, keyLen);
  }

  // =============================================================
  // TELEMETRY (same as previous)
  // =============================================================

  var MAX_PTS = 600;
  var livePktCount = 0, lastPktMs = 0, serverUptimeMs = 0, serverSyncClockMs = 0;
  var rktUptimeMs = 0, rktSyncClockMs = 0;  // rocket's own boot clock (from status char)
  var fullGpsLat = null, fullGpsLon = null, pages = {};
  // Two separate live session stores. viewIdx=-1 means live; liveSource='base' or 'rkt'.
  var sessions = [], viewIdx = -1, liveSource = 'base';
  var baseLiveRecs = [], rktLiveRecs = [], fetchAbort = false;
  var allFetched = {}, fetchedLowest = Infinity, fetchedHighest = -1;
  var dlSession = []; // LoRa download session — all 0xCA records accumulate here
  var fetchSpeedRecs = 0, fetchSpeedBytes = 0, fetchSpeedStartMs = 0;
  var fetchAutoNav = true; // auto-navigate to latest session after each batch
  var dlSpeedRecs = 0, dlSpeedBytes = 0, dlSpeedStartMs = 0;
  var PHASES = ['idle','pad_ready','boost','coast','apogee','drogue','main_deploy','descent','landed','ground_test','sleep','startup','12','13','14','error'];
  var LR_HISTORY_COUNT = 10;
  var LR_MAX_PTS = 1000;
  var lrHistory = [];

  function pad2(n){return n<10?'0'+n:''+n}
  function pad3(n){return n<10?'00'+n:n<100?'0'+n:''+n}
  function fmtBoot(ms){if(ms<0)ms=0;var s=Math.floor(ms/1000);var m=Math.floor(s/60);s%=60;var h=Math.floor(m/60);m%=60;return pad2(h)+':'+pad2(m)+':'+pad2(s)+'.'+pad3(ms%1000)}
  function fmtBootShort(ms){if(ms<0)ms=0;var s=Math.floor(ms/1000);var m=Math.floor(s/60);s%=60;var h=Math.floor(m/60);m%=60;return h>0?h+':'+pad2(m)+':'+pad2(s):m+':'+pad2(s)}
  function toHex(arr){var h='';for(var i=0;i<arr.length;i++)h+=arr[i].toString(16).padStart(2,'0');return h}

  var PD = {};
  PD[1]={n:'GPS Full',s:10,d:function(v,o){return{lat:v.getInt32(o,1)/1e7,lon:v.getInt32(o+4,1)/1e7,hdop:v.getUint8(o+8)/10,sats:v.getUint8(o+9)}}};
  PD[2]={n:'Baro',s:6,d:function(v,o){var r={alt_cm:v.getInt32(o,1),vvel:v.getInt16(o+4,1)/10};if(v.byteLength>=o+8)r.gnd=v.getInt16(o+6,1);return r},f:function(d){var s=(d.alt_cm/100).toFixed(1)+'m MSL vv:'+d.vvel.toFixed(1);if(d.gnd!==undefined)s+=' gnd:'+d.gnd+'m';return s}};
  PD[3]={n:'Mag',s:6,d:function(v,o){return{x:v.getInt16(o,1),y:v.getInt16(o+2,1),z:v.getInt16(o+4,1)}},f:function(d){return d.x+' '+d.y+' '+d.z+' mG'}};
  PD[4]={n:'Accel',s:6,d:function(v,o){return{x:v.getInt16(o,1),y:v.getInt16(o+2,1),z:v.getInt16(o+4,1)}},f:function(d){return d.x+' '+d.y+' '+d.z+' mg'}};
  PD[5]={n:'Gyro',s:6,d:function(v,o){return{x:v.getInt16(o,1)/10,y:v.getInt16(o+2,1)/10,z:v.getInt16(o+4,1)/10}},f:function(d){return d.x.toFixed(1)+' '+d.y.toFixed(1)+' '+d.z.toFixed(1)+' d/s'}};
  PD[6]={n:'GPS Ext',s:10,d:function(v,o){return{gspd:v.getUint16(o,1)/100,crs:v.getUint16(o+2,1)/100,fixAge:v.getUint8(o+4),vdop:v.getUint8(o+5)/10,alt_cm:v.getInt32(o+6,1)}},f:function(d){return d.gspd.toFixed(1)+'m/s '+d.crs.toFixed(0)+'° age:'+d.fixAge+' V:'+d.vdop.toFixed(1)+' '+(d.alt_cm/100).toFixed(1)+'m'}};
  PD[7]={n:'Kalman',s:10,d:function(v,o){return{alt_cm:v.getInt32(o,1),vel:v.getInt16(o+4,1)/10,altUnc:v.getUint8(o+6),velUnc:v.getUint8(o+7),accUnc:v.getUint8(o+8),innov:v.getUint8(o+9)}},f:function(d){return(d.alt_cm/100).toFixed(1)+'m v:'+d.vel.toFixed(1)+' u:'+d.altUnc+'/'+d.velUnc}};
  PD[8]={n:'System',s:11,d:function(v,o){return{temp:v.getInt8(o),heap:v.getUint16(o+1,1),up:v.getUint16(o+3,1),batt:v.getUint16(o+5,1),logRec:v.getUint32(o+7,1)}},f:function(d){return d.temp+'C '+d.batt+'mV '+d.heap+'KB '+d.up+'s log#'+d.logRec}};
  PD[9]={n:'Peaks',s:8,d:function(v,o){return{alt:v.getInt32(o,1),acc:v.getUint16(o+4,1)/100,vvel:v.getInt16(o+6,1)/10}},f:function(d){return(d.alt/100).toFixed(1)+'m '+d.acc.toFixed(1)+'g vv:'+d.vvel.toFixed(1)}};
  PD[10]={n:'Cmd Ack',s:10,d:function(v,o){return{nonce:v.getUint32(o,1),res:v.getUint8(o+4),rssi:v.getInt8(o+5),snr:v.getInt8(o+6)/4,hmacFail:v.getUint16(o+7,1),rxPosInSlot:v.getUint8(o+9)}},f:function(d){return 'N:'+d.nonce+(d.res===0?' OK':' E'+d.res)+' rssi:'+d.rssi+' snr:'+d.snr.toFixed(1)+' hmF:'+d.hmacFail+' rxPos:'+d.rxPosInSlot+'*2ms'}};
  PD[11]={n:'Flight',s:6,d:function(v,o){var ms=v.getInt32(o,1);var fl=v.getUint16(o+4,1);return{ms:ms,p1c:!!(fl&1),p1f:!!(fl&2),p1a:!!(fl&4),p2c:!!(fl&8),p2f:!!(fl&16),p2a:!!(fl&32),cc:!!(fl&64),cf:!!(fl&128),ca:!!(fl&256)}},f:function(d){return(d.ms<0?'pre':'T+'+(d.ms/1000).toFixed(1)+'s')+' P1:'+(d.p1c?'*':'o')+(d.p1f?'F':'')+' P2:'+(d.p2c?'*':'o')+(d.p2f?'F':'')+' Ch:'+(d.cc?'*':'o')+(d.cf?'F':'')}};
  PD[12]={n:'Radio',s:6,d:function(v,o){var f=v.getUint8(o+5);return{dtx:v.getUint16(o,1),irx:v.getUint16(o+2,1),bgRssi:v.getInt8(o+4),radioSynced:!!(f&1),seqIdx:(f>>1)&0x7F}},f:function(d){return 'dTX:'+d.dtx+' bRX:'+d.irx+' bg:'+d.bgRssi+'dBm sync:'+(d.radioSynced?'Y':'N')+' seq:'+d.seqIdx}};
  PD[13]={n:'Time',s:8,d:function(v,o){var lo=v.getUint32(o,1),hi=v.getUint32(o+4,1);var ms=hi*4294967296+lo;return{ms:ms,utc:ms>0?new Date(ms):null}},f:function(d){return d.utc?d.utc.toISOString().replace('T',' ').substring(0,19)+'Z':'no time'}};
  // Page 0x0E: Thrust Curve — variable-length accel ring buffer snapshot
  // Header: durationMs u16, minAccel8 s16, maxAccel8 s16 (8mg units), then N sample bytes.
  // Sample count inferred from totalLen - 6 (totalLen passed in for BLE records; falls back to DataView length).
  PD[14]={n:'Thrust Curve',s:6,d:function(v,o,totalLen){
    var durationMs=v.getUint16(o,true);
    var minMg8=v.getInt16(o+2,true);
    var maxMg8=v.getInt16(o+4,true);
    var nSamples=(totalLen!==undefined?totalLen:(v.byteLength-o))-6;
    if(nSamples<0)nSamples=0;
    var samples=new Uint8Array(v.buffer,v.byteOffset+o+6,nSamples);
    return{durationMs:durationMs,minMg:minMg8*8,maxMg:maxMg8*8,sampleCount:nSamples,samples:samples};
  },f:function(d){return d.sampleCount+' samples, '+d.durationMs+'ms, '+d.minMg+'..'+d.maxMg+' mg'}};
  // Page 0x0F: Pyro Status (6 bytes)
  // flags u8: [0]=ch1_cont [1]=ch2_cont [2]=ch3_cont [3]=hv_present [4]=ch1_fired [5]=ch2_fired [6]=ch3_fired [7]=active
  // active_channel u8, active_duration_ms u16, hv_millivolts u16
  PD[15]={n:'Pyro',s:6,d:function(v,o){
    var fl=v.getUint8(o);
    return{
      ch1_cont:!!(fl&1),ch2_cont:!!(fl&2),ch3_cont:!!(fl&4),
      hv_present:!!(fl&8),
      ch1_fired:!!(fl&16),ch2_fired:!!(fl&32),ch3_fired:!!(fl&64),
      active:!!(fl&128),
      active_ch:v.getUint8(o+1),
      dur_ms:v.getUint16(o+2,true),
      hv_mv:v.getUint16(o+4,true)
    };
  },f:function(d){
    var cont=(d.ch1_cont?'1':'_')+(d.ch2_cont?'2':'_')+(d.ch3_cont?'3':'_');
    var fired=(d.ch1_fired?'1':'_')+(d.ch2_fired?'2':'_')+(d.ch3_fired?'3':'_');
    var hv=d.hv_present?('HV:'+d.hv_mv+'mV'):'no-HV';
    var act=d.active?('ch'+d.active_ch+' '+d.dur_ms+'ms'):'idle';
    return 'cont:'+cont+' fired:'+fired+' '+hv+' '+act;
  }};

  var charts = null;
  function makeChart(id, ds) {
    return new Chart(document.getElementById(id), {
      type:'line', data:{datasets:ds.map(function(d){return{label:d.l,data:[],borderColor:d.c,backgroundColor:d.c,pointRadius:1.1,fill:false,tension:0,borderWidth:0.5}})},
      options:{animation:false,responsive:true,maintainAspectRatio:false,
        scales:{x:{type:'linear',display:true,title:{display:true,text:'T+ boot',color:'#666',font:{size:10}},ticks:{color:'#666',font:{size:9},maxTicksLimit:8,callback:function(v){return fmtBootShort(v)}},grid:{color:'#1a1a1a'}},y:{ticks:{color:'#888',font:{size:10}},grid:{color:'#222'}}},
        plugins:{legend:{labels:{color:'#aaa',font:{size:10},boxWidth:12},position:'top'},tooltip:{callbacks:{title:function(i){return i.length?'T+'+fmtBoot(i[0].parsed.x):''}}}}}
    });
  }
function initCharts() {
    if (typeof Chart === 'undefined') return false;
    charts = {
      alt:  makeChart('cAlt', [{l:'Fusion',c:'rgba(255,136,0,0.75)'},{l:'GPS',c:'rgba(0,255,0,0.75)'},{l:'Baro',c:'rgba(0,170,255,0.75)'}]),
      snr:  makeChart('cSNR', [{l:'SNR',c:'rgba(255,255,0,0.75)'},{l:'RSSI',c:'rgba(255,136,0,0.75)'},{l:'BG RSSI',c:'rgba(255,68,68,0.75)'}]),
      acc:  makeChart('cAcc', [{l:'Acc X',c:'rgba(255,68,68,0.75)'},{l:'Acc Y',c:'rgba(68,255,68,0.75)'},{l:'Acc Z',c:'rgba(68,68,255,0.75)'}]),
      gyr:  makeChart('cGyr', [{l:'Gyr X',c:'rgba(255,68,68,0.75)'},{l:'Gyr Y',c:'rgba(68,255,68,0.75)'},{l:'Gyr Z',c:'rgba(68,68,255,0.75)'}]),
      mag:  makeChart('cMag', [{l:'Mag X',c:'rgba(255,68,68,0.75)'},{l:'Mag Y',c:'rgba(68,255,68,0.75)'},{l:'Mag Z',c:'rgba(68,68,255,0.75)'}]),
      batt: makeChart('cBatt',[{l:'Rocket mV',c:'rgba(0,255,0,0.75)'},{l:'Base mV',c:'rgba(255,255,0,0.75)'}])
    };
    makeThrustChart();
    return true;
  }
  function clearCharts(){if(!charts)return;for(var k in charts){var c=charts[k];for(var i=0;i<c.data.datasets.length;i++)c.data.datasets[i].data=[];delete c.options.scales.x.min;delete c.options.scales.x.max;c.update('none')}}
  function pushChart(ch,t,v){if(!ch)return;for(var i=0;i<v.length;i++){if(v[i]!==null&&v[i]!==undefined)ch.data.datasets[i].data.push({x:t,y:v[i]})}for(var j=0;j<ch.data.datasets.length;j++){var d=ch.data.datasets[j].data;while(d.length>MAX_PTS)d.shift()}ch.update('none')}

  // Thrust curve chart — independent from the telemetry time-series charts.
  // x-axis is time within the captured window (0..durationMs), not boot time.
  // Does not participate in cross-chart scroll/zoom sync.
  var thrustChart = null;
  function makeThrustChart() {
    if (typeof Chart === 'undefined') return;
    var el = document.getElementById('cThrust');
    if (!el) return;
    thrustChart = new Chart(el, {
      type: 'line',
      data: {datasets:[{label:'Thrust Curve (Whoosh!)',data:[],borderColor:'rgba(255,136,0,0.85)',backgroundColor:'rgba(255,136,0,0.15)',pointRadius:0,fill:true,tension:0,borderWidth:1}]},
      options: {
        animation: false, responsive: true, maintainAspectRatio: false,
        scales: {
          x: {type:'linear',display:true,title:{display:true,text:'Time (s)',color:'#666',font:{size:10}},ticks:{color:'#666',font:{size:9},maxTicksLimit:8},grid:{color:'#1a1a1a'}},
          y: {ticks:{color:'#888',font:{size:10}},grid:{color:'#222'},title:{display:true,text:'g',color:'#666',font:{size:10}}}
        },
        plugins:{legend:{labels:{color:'#888',font:{size:10}}}}
      }
    });
  }
  function clearThrustChart() {
    if (!thrustChart) return;
    thrustChart.data.datasets[0].data = [];
    delete thrustChart.options.scales.x.min;
    delete thrustChart.options.scales.x.max;
    delete thrustChart.options.scales.y.min;
    delete thrustChart.options.scales.y.max;
    thrustChart.update('none');
  }
  function loadThrustCurve(d) {
    if (!thrustChart) return;
    var pts = [];
    var N = d.sampleCount;
    var range = d.maxMg - d.minMg;
    for (var i = 0; i < N; i++) {
      var x = N > 1 ? (i * d.durationMs / (N - 1)) / 1000 : 0;
      var y = (d.minMg + (d.samples[i] / 255) * range) / 1000;
      pts.push({x: x, y: y});
    }
    thrustChart.data.datasets[0].data = pts;
    thrustChart.options.scales.x.min = -0.01;
    thrustChart.options.scales.x.max = d.durationMs / 1000 + 0.01;
    thrustChart.options.scales.y.min = d.minMg / 1000;
    thrustChart.options.scales.y.max = d.maxMg / 1000;
    thrustChart.update();
  }

  // Per-page detailed field descriptions for expanded view
  var PF = {};
  PF['hdr']=[{k:'dev',n:'Device',u:''},{k:'alt',n:'Alt',u:'m MSL (fusion, int16)'},{k:'latF',n:'Lat frac',u:''},{k:'lonF',n:'Lon frac',u:''},{k:'phase',n:'Phase',u:''},{k:'armed',n:'Armed',u:'bool'},{k:'ch1_fired',n:'Ch1 fired',u:'bool'},{k:'ch2_fired',n:'Ch2 fired',u:'bool'},{k:'ch3_fired',n:'Ch3 fired',u:'bool'},{k:'low_batt',n:'LowBatt',u:'bool'},{k:'rsv_9_15',n:'Rsv[9:15]',u:'bin'}];
  PF[1]=[{k:'lat',n:'Lat',u:'°',fmt:function(v){return v.toFixed(7)}},{k:'lon',n:'Lon',u:'°',fmt:function(v){return v.toFixed(7)}},{k:'hdop',n:'HDOP',u:'',fmt:function(v){return v.toFixed(1)}},{k:'sats',n:'Sats',u:''}];
  PF[2]=[{k:'alt_cm',n:'Alt MSL',u:'m',fmt:function(v){return(v/100).toFixed(1)}},{k:'vvel',n:'VVel',u:'m/s',fmt:function(v){return v.toFixed(1)}},{k:'gnd',n:'Ground',u:'m',fmt:function(v){return v===undefined?'--':''+v}}];
  PF[3]=[{k:'x',n:'X',u:'mG'},{k:'y',n:'Y',u:'mG'},{k:'z',n:'Z',u:'mG'}];
  PF[4]=[{k:'x',n:'X',u:'mg'},{k:'y',n:'Y',u:'mg'},{k:'z',n:'Z',u:'mg'}];
  PF[5]=[{k:'x',n:'X',u:'°/s',fmt:function(v){return v.toFixed(1)}},{k:'y',n:'Y',u:'°/s',fmt:function(v){return v.toFixed(1)}},{k:'z',n:'Z',u:'°/s',fmt:function(v){return v.toFixed(1)}}];
  PF[6]=[{k:'gspd',n:'GSpd',u:'m/s',fmt:function(v){return v.toFixed(1)}},{k:'crs',n:'Crs',u:'°',fmt:function(v){return v.toFixed(0)}},{k:'fixAge',n:'Age',u:'×100ms'},{k:'vdop',n:'VDOP',u:'',fmt:function(v){return v.toFixed(1)}},{k:'alt_cm',n:'Alt',u:'m MSL',fmt:function(v){return(v/100).toFixed(1)}}];
  PF[7]=[{k:'alt_cm',n:'Alt',u:'m AGL',fmt:function(v){return(v/100).toFixed(1)}},{k:'vel',n:'Vel',u:'m/s',fmt:function(v){return v.toFixed(1)}},{k:'altUnc',n:'AltU',u:'log'},{k:'velUnc',n:'VelU',u:'log'},{k:'accUnc',n:'AccU',u:'log'},{k:'innov',n:'Inn',u:'log'}];
  PF[8]=[{k:'temp',n:'Temp',u:'°C'},{k:'heap',n:'Heap',u:'KB'},{k:'up',n:'Up',u:'s'},{k:'batt',n:'Batt',u:'mV'},{k:'logRec',n:'Log#',u:''}];
  PF[9]=[{k:'alt',n:'MaxAlt',u:'m',fmt:function(v){return(v/100).toFixed(1)}},{k:'acc',n:'MaxAcc',u:'g',fmt:function(v){return v.toFixed(2)}},{k:'vvel',n:'MaxVV',u:'m/s',fmt:function(v){return v.toFixed(1)}}];
  PF[10]=[{k:'nonce',n:'Nonce',u:''},{k:'res',n:'Result',u:'0=OK'},{k:'rssi',n:'RSSI',u:'dBm'},{k:'snr',n:'SNR',u:'dB',fmt:function(v){return v.toFixed(1)}},{k:'hmacFail',n:'HmacF',u:'count'},{k:'rxPosInSlot',n:'RxPos',u:'ms',fmt:function(v){return(v*2)}}];
  PF[11]=[{k:'ms',n:'T+launch',u:'ms'},{k:'p1c',n:'P1cont',u:'bool'},{k:'p1f',n:'P1fire',u:'bool'},{k:'p1a',n:'P1act',u:'bool'},{k:'p2c',n:'P2cont',u:'bool'},{k:'p2f',n:'P2fire',u:'bool'},{k:'p2a',n:'P2act',u:'bool'},{k:'cc',n:'Chcont',u:'bool'},{k:'cf',n:'Chfire',u:'bool'},{k:'ca',n:'Chact',u:'bool'}];
  PF[12]=[{k:'dtx',n:'DlyTX',u:'count'},{k:'irx',n:'BadRX',u:'count'},{k:'bgRssi',n:'BgRSSI',u:'dBm'},{k:'radioSynced',n:'Sync',u:'Y/N'},{k:'seqIdx',n:'SeqIdx',u:'slot'}];
  PF[13]=[{k:'utc',n:'UTC',u:'',fmt:function(v){return v?v.toISOString().replace('T',' ').substring(0,19)+'Z':'none'}}];
  PF[14]=[{k:'durationMs',n:'Duration',u:'ms'},{k:'minMg',n:'Min',u:'mg'},{k:'maxMg',n:'Max',u:'mg'},{k:'sampleCount',n:'Samples',u:''}];

  var detailMode = false;
  var lastPageData = {}; // store latest decoded data per page key

  function initGrid(){
    var g=document.getElementById('grid');
    var h='<div class="card" id="card-hdr"><div class="cl">Main</div><div class="cv" id="v-hdr">&mdash;</div><div class="cd" id="det-hdr"></div><div class="ad" id="d-hdr"></div></div>';
    var ids=Object.keys(PD).map(Number).sort(function(a,b){return a-b});
    for(var i=0;i<ids.length;i++){
      var id=ids[i];
      var extra='';
      if(id===1) extra='<div class="gps-row" id="gps-lnk" style="display:none"><a id="gps-map" href="#" target="_blank">Map</a><button class="cpb" id="gps-cp">Copy</button></div>';
      h+='<div class="card" id="card-'+id+'"><div class="cl">0x'+id.toString(16).padStart(2,'0')+' '+PD[id].n+'</div><div class="cv" id="v-'+id+'">&mdash;</div>'+extra+'<div class="cd" id="det-'+id+'"></div><div class="ad" id="d-'+id+'"></div></div>';
    }
    h+='<div class="card" id="card-lr"><div class="cl">0xBB Long Range</div><div class="cv" id="v-lr">&mdash;</div><div class="gps-row" id="lr-lnk" style="display:none"><a id="lr-map" href="#" target="_blank">Map</a><button class="cpb" id="lr-cp">Copy</button></div><div class="cd" id="det-lr"></div><div class="ad" id="d-lr"></div></div>';
    g.innerHTML=h;
    document.getElementById('gps-cp').addEventListener('click',function(){
      if(fullGpsLat!==null){var txt=fullGpsLat.toFixed(7)+','+fullGpsLon.toFixed(7);if(navigator.clipboard&&window.isSecureContext){navigator.clipboard.writeText(txt)}else{var ta=document.createElement('textarea');ta.value=txt;ta.style.position='fixed';ta.style.left='-9999px';document.body.appendChild(ta);ta.select();try{document.execCommand('copy')}catch(e){}document.body.removeChild(ta)}this.textContent='OK';var s=this;setTimeout(function(){s.textContent='Copy'},1500)}
    });
    var lrCpBtn=document.getElementById('lr-cp');if(lrCpBtn){lrCpBtn.addEventListener('click',function(){if(lrHistory.length){var lastN=Math.min(10,lrHistory.length),avgLat=0,avgLon=0,cnt=0;for(var j=lrHistory.length-lastN;j<lrHistory.length;j++){var lf=lrHistory[j].latFrac,ln=lrHistory[j].lonFrac;if(lf<2000&&ln<2000){avgLat+=lf;avgLon+=ln;cnt++}}if(cnt>0){var txt='x.'+(avgLat/cnt/2000).toFixed(6).slice(2)+',x.'+(avgLon/cnt/2000).toFixed(6).slice(2);if(navigator.clipboard&&window.isSecureContext){navigator.clipboard.writeText(txt)}else{var ta=document.createElement('textarea');ta.value=txt;ta.style.position='fixed';ta.style.left='-9999px';document.body.appendChild(ta);ta.select();try{document.execCommand('copy')}catch(e){}document.body.removeChild(ta)}this.textContent='OK';var s=this;setTimeout(function(){s.textContent='Copy'},1500)}}})}
  }

  function formatField(f,v){
    if(v===undefined||v===null)return'--';
    if(f.fmt)return f.fmt(v);
    if(typeof v==='boolean')return v?'Y':'N';
    return''+v;
  }

  function updateDetail(key,data){
    lastPageData[key]=data;
    if(!detailMode)return;
    showDetail(key,data);
  }

  function showDetail(key,data){
    var el=document.getElementById('det-'+key);
    if(!el||!data)return;
    var fields=PF[key];
    if(!fields){el.innerHTML='';return;}
    var lines=[];
    for(var i=0;i<fields.length;i++){
      var f=fields[i],v=data[f.k];
      lines.push('<span class="df"><span class="dn">'+f.n+':</span> <span class="dv">'+formatField(f,v)+'</span>'+(f.u?' <span class="du">'+f.u+'</span>':'')+'</span>');
    }
    el.innerHTML=lines.join('');
  }

  function toggleDetail(){
    detailMode=!detailMode;
    document.getElementById('btn-detail').className=detailMode?'act':'';
    var g=document.getElementById('grid');
    if(detailMode)g.classList.add('detail-on');else g.classList.remove('detail-on');
    var dets=document.querySelectorAll('.cd');
    for(var i=0;i<dets.length;i++)dets[i].style.display=detailMode?'block':'none';
    if(detailMode){for(var k in lastPageData)showDetail(k,lastPageData[k])}
  }
  var pageTimes = {};
  function setV(k,t,isLive){var e=document.getElementById('v-'+k);if(e)e.textContent=t;if(isLive)pageTimes[k]=Date.now()}
  function resolveGps(lf,lnf){if(lf>=50000||lnf>=50000||fullGpsLat===null)return null;var la=Math.floor(Math.abs(fullGpsLat))+lf*0.00002;var lo=Math.floor(Math.abs(fullGpsLon))+lnf*0.00002;if(fullGpsLat<0)la=-la;if(fullGpsLon<0)lo=-lo;return{lat:la,lon:lo}}
  function lerpColor(a,b,t){t=Math.max(0,Math.min(1,t));var r1=parseInt(a.slice(1,3),16),g1=parseInt(a.slice(3,5),16),b1=parseInt(a.slice(5,7),16);var r2=parseInt(b.slice(1,3),16),g2=parseInt(b.slice(3,5),16),b2=parseInt(b.slice(5,7),16);return '#'+('0'+Math.round(r1+(r2-r1)*t).toString(16)).slice(-2)+('0'+Math.round(g1+(g2-g1)*t).toString(16)).slice(-2)+('0'+Math.round(b1+(b2-b1)*t).toString(16)).slice(-2)}
  function tickDots(){var now=Date.now();for(var k in pageTimes){var age=(now-pageTimes[k])/1000;var dot=document.getElementById('d-'+k);if(!dot)continue;var c;if(age<2)c=lerpColor('#00ff00','#ffff00',(age)/2);else if(age<15)c=lerpColor('#ffff00','#ff4400',(age-2)/13);else if(age<60)c=lerpColor('#ff4400','#ff0000',(age-15)/45);else c='#444444';dot.style.background=c;dot.style.opacity=Math.max(0.3,Math.exp(-age/30))}var rd=document.getElementById('dot-rkt'),rv=document.getElementById('val-rkt');if(!livePktCount){rd.className='dot dd';rv.textContent='no pkts'}else{var a=(Date.now()-lastPktMs)/1000;if(a<3){rd.className='dot dg';rv.textContent=a.toFixed(2)+'s'}else if(a<15){rd.className='dot dy';rv.textContent=a.toFixed(1)+'s'}else{rd.className='dot dr';rv.textContent=a.toFixed(1)+'s'}}}
  setInterval(tickDots, 100);
  function setWs(on){document.getElementById('dot-ws').className='dot '+(on?'dg':'dr');document.getElementById('val-ws').textContent=on?'connected':'disconnected'}
  function setSNR(s){var e=document.getElementById('val-snr-top');if(s===127){e.textContent='--';e.style.color='#555';return}e.textContent=s.toFixed(1)+'dB';e.style.color=s>5?'#0f0':s>0?'#ff0':s>-5?'#f80':'#f44'}

  function decodeTelem(buf){var dv=new DataView(buf);if(dv.byteLength<10||dv.getUint8(0)!==0xAF)return null;var r={dev:dv.getUint8(1),latF:dv.getUint16(2,1),lonF:dv.getUint16(4,1),altM:dv.getInt16(6,1),flags:dv.getUint16(8,1),pg:null};if(dv.byteLength>10){var pt=dv.getUint8(10);var df=PD[pt];if(df&&dv.byteLength>=11+df.s){var pgTotalLen=dv.byteLength-11;r.pg={t:pt,d:df.d(dv,11,pgTotalLen)}}}return r}
  function decodeLR(buf){var dv=new DataView(buf);if(dv.byteLength<5||dv.getUint8(0)!==0xBB)return null;var word=dv.getUint8(2)|(dv.getUint8(3)<<8)|(dv.getUint8(4)<<16);var latFrac=word&0x7FF;var lonFrac=(word>>11)&0x7FF;var lowBatt=(word>>22)&1;var reserved=(word>>23)&1;if(latFrac>=2000||lonFrac>=2000)return null;return{latFrac:latFrac,lonFrac:lonFrac,lowBatt:!!lowBatt,reserved:!!reserved}}
  function fmtGpsFrac(v){if(v===65535)return'NOT_POWERED';if(v===65534)return'INIT';if(v===65533)return'NO_FIX';if(v>=50000)return'Err:'+v;return'#.'+(v*2+'').padStart(5,'0')}
  function showPkt(buf,snr,rssi,bootMs,toC,isLive){var p=decodeTelem(buf);if(!p)return;var ph=p.flags&0xF,arm=!!(p.flags&0x10),lok=p.latF<50000,aok=p.altM!==-32768;var ch1f=!!(p.flags&0x20),ch2f=!!(p.flags&0x40),ch3f=!!(p.flags&0x80),lowb=!!(p.flags&0x100),rsv=(p.flags>>9)&0x7F;var hdrParts=[];hdrParts.push('dev:'+p.dev);if(aok)hdrParts.push('alt:'+p.altM+'m');hdrParts.push('lat:'+fmtGpsFrac(p.latF)+' lon:'+fmtGpsFrac(p.lonF));hdrParts.push(PHASES[ph].toUpperCase());if(arm)hdrParts.push('ARM');if(ch1f)hdrParts.push('CH1');if(ch2f)hdrParts.push('CH2');if(ch3f)hdrParts.push('CH3');if(lowb)hdrParts.push('LOWBAT');setV('hdr',hdrParts.join(' '),isLive);updateDetail('hdr',{dev:p.dev,alt:aok?p.altM:'--',latF:fmtGpsFrac(p.latF),lonF:fmtGpsFrac(p.lonF),phase:PHASES[ph],armed:arm,ch1_fired:ch1f,ch2_fired:ch2f,ch3_fired:ch3f,low_batt:lowb,rsv_9_15:'0b'+(rsv).toString(2).padStart(7,'0')});document.getElementById('phase').textContent=PHASES[ph].toUpperCase();document.getElementById('phase').className='st-phase p-'+PHASES[ph];var fusAlt=aok?p.altM:null,baroAlt=null,gpsAlt=null,bgRssi=null;if(p.pg){var t=p.pg.t,d=p.pg.d;if(t===1){fullGpsLat=d.lat;fullGpsLon=d.lon;document.getElementById('gps-map').href='https://www.google.com/maps?q='+d.lat.toFixed(7)+','+d.lon.toFixed(7);document.getElementById('gps-lnk').style.display='flex'}var df=PD[t];if(df&&df.f)setV(t,df.f(d),isLive);updateDetail(t,d);if(t===2)baroAlt=d.alt_cm/100;if(t===6)gpsAlt=d.alt_cm/100;if(t===12)bgRssi=d.bgRssi;if(t===3&&toC&&charts)pushChart(charts.mag,bootMs,[d.x,d.y,d.z]);if(t===4&&toC&&charts)pushChart(charts.acc,bootMs,[d.x,d.y,d.z]);if(t===5&&toC&&charts)pushChart(charts.gyr,bootMs,[d.x,d.y,d.z]);if(t===8&&toC&&charts)pushChart(charts.batt,bootMs,[d.batt,null]);if(t===14)loadThrustCurve(d)}if(toC&&charts){pushChart(charts.alt,bootMs,[fusAlt,gpsAlt,baroAlt]);pushChart(charts.snr,bootMs,[snr,rssi,bgRssi])}if(isLive&&voiceEnabled)voiceOnTelem(p,ph,arm,aok?p.altM:null)}

  function getLiveRecs(){return liveSource==='rkt'?rktLiveRecs:baseLiveRecs}
  function updateNav(){var n=sessions.length;document.getElementById('btn-prev').disabled=(viewIdx===-1&&n===0)||(viewIdx===0);document.getElementById('btn-next').disabled=(viewIdx===-1)||(viewIdx>=n-1);var lbl=liveSource==='rkt'?'Live Rkt':'Live Base';var info;if(viewIdx===-1){info=lbl+(n?' | '+n+'sess':'')}else{var isDl=sessions[viewIdx]===dlSession;info=(viewIdx+1)+'/'+n+(isDl?' DL':'')+' ('+sessions[viewIdx].length+')'}document.getElementById('sess-info').textContent=info;var bLive=document.getElementById('btn-live-base'),rLive=document.getElementById('btn-live-rkt');if(bLive)bLive.className=viewIdx===-1&&liveSource==='base'?'act':'';if(rLive)rLive.className=viewIdx===-1&&liveSource==='rkt'?'act':'';}
  function showView(idx){viewIdx=idx;clearCharts();document.getElementById('log').innerHTML='';updateNav();var recs=idx===-1?getLiveRecs():sessions[idx];for(var i=0;i<recs.length;i++){var r=recs[i];showRecord(r,true,false);addLogRec(r,idx!==-1)}}
  function addLogRec(rec,hist){
    var bytes=new Uint8Array(rec.payload);var hex=toHex(bytes);
    var el=document.getElementById('log');var e=document.createElement('div');e.className='le';
    var prefix=hist?'H':'L';
    if(rec.type==='logpage'){prefix='DL';e.style.color='#a8f'}
    e.textContent=prefix+' #'+rec.recNum+' T+'+fmtBoot(rec.ts)+' snr:'+(rec.snr||0).toFixed(1)+' ['+hex+']';
    if(hist)el.appendChild(e);else el.insertBefore(e,el.firstChild);
    while(el.children.length>1000)el.removeChild(hist?el.firstChild:el.lastChild);
  }
  function onLivePkt(buf,snr,rssi,recNum){livePktCount++;lastPktMs=Date.now();document.getElementById('val-cnt').textContent=livePktCount;if(recNum>=0)document.getElementById('val-logrec').textContent=recNum;setSNR(snr);var bootMs=serverUptimeMs+(Date.now()-serverSyncClockMs);var rec={recNum:recNum,snr:snr,rssi:rssi,ts:bootMs,payload:buf,type:'pkt'};baseLiveRecs.push(rec);if(baseLiveRecs.length>MAX_PTS)baseLiveRecs.shift();showPkt(buf,snr,rssi,bootMs,viewIdx===-1&&liveSource==='base',true);if(viewIdx===-1&&liveSource==='base'){addLogRec(rec,false)}mapAddPt(buf,snr,rssi,recNum);if(new Uint8Array(buf)[0]===0xBB)onLRPkt(buf,snr,rssi,recNum);}
  function onLRPkt(buf,snr,rssi,recNum){var p=decodeLR(buf);if(!p)return;p.wallTime=Date.now();p.snr=snr;p.rssi=rssi;lrHistory.push(p);if(lrHistory.length>LR_MAX_PTS)lrHistory.shift();updateLRCard(true);mapScheduleDraw()}
  function updateLRCard(isLive){if(!lrHistory.length)return;var sumLat=0,sumLon=0,cntL=0,cntS=0,validCnt=0;for(var i=0;i<lrHistory.length;i++){var lf=lrHistory[i].latFrac,ln=lrHistory[i].lonFrac;if(lf<2000&&ln<2000){sumLat+=lf;sumLon+=ln;validCnt++}if(lrHistory[i].lowBatt)cntL++;if(lrHistory[i].reserved)cntS++}var n=lrHistory.length;var pctL=String(Math.round(cntL/n*100)).padStart(2,'0');var pctS=String(Math.round(cntS/n*100)).padStart(2,'0');function fmtFrac(v){return'x.'+(v/2000).toFixed(6).slice(2)}var mainTxt=validCnt>0?(fmtFrac(sumLat/validCnt)+','+fmtFrac(sumLon/validCnt)+' L'+pctL+' S'+pctS):'no valid';setV('lr',mainTxt,isLive);if(validCnt>0&&fullGpsLat!==null){var mapLat=Math.trunc(fullGpsLat)+(sumLat/validCnt/2000)*Math.sign(fullGpsLat);var mapLon=Math.trunc(fullGpsLon)+(sumLon/validCnt/2000)*Math.sign(fullGpsLon);document.getElementById('lr-map').href='https://www.google.com/maps?q='+mapLat.toFixed(6)+','+mapLon.toFixed(6);document.getElementById('lr-lnk').style.display='flex'}else{document.getElementById('lr-lnk').style.display='none'}var lines='';var lastN=Math.min(10,lrHistory.length);for(var j=lrHistory.length-1;j>=lrHistory.length-lastN;j--){var e=lrHistory[j];var lf=e.latFrac,ln=e.lonFrac;var lFlag=e.lowBatt?'L':'-';var sFlag=e.reserved?'S':'-';var txt=lf>=2000||ln>=2000?('err:'+lf+','+ln+' '+lFlag+sFlag):fmtFrac(lf)+','+fmtFrac(ln)+' '+lFlag+sFlag;lines+='<span class="df"><span class="dv">'+txt+'</span></span>'}var det=document.getElementById('det-lr');if(det)det.innerHTML=lines}

  // =============================================================
  // 0xCA LOG CHUNK DECODER (LoRa download from rocket)
  // =============================================================
  // Chunk format: [0xCA][srcDevID][startRecIdx u32 LE][recCount u8][concat flash records]
  // Flash record: [length u8][snr i8][timestamp u32 LE][payload: length bytes]
  // Payload starts with type byte: <=0x7F = raw data page, >=0x80 = received packet

  function onLogChunk(buf, wsSnr, wsRssi, bsRecNum) {
    var bytes = new Uint8Array(buf);
    if (bytes.length < 8 || bytes[0] !== 0xCA) return;
    var dv = new DataView(buf);
    var srcDev = bytes[1];
    var startIdx = dv.getUint32(2, true);
    var recCount = bytes[6];
    var off = 7;
    var added = 0;

    for (var i = 0; i < recCount && off < bytes.length; i++) {
      if (off + 6 > bytes.length) break; // need at least log header
      var pLen = bytes[off];
      if (pLen === 0xFF || pLen === 0 || pLen > 254) break; // erased/end marker
      var logSnr = (bytes[off + 1] << 24) >> 24; // sign-extend i8
      var logTs = dv.getUint32(off + 2, true);
      off += 6; // skip log header
      if (off + pLen > bytes.length) break;

      var payload = buf.slice(off, off + pLen);
      off += pLen;

      var rocketRecNum = startIdx + i;
      // Determine record type from first byte of payload
      var firstByte = new Uint8Array(payload)[0];
      var recType = (firstByte >= 0x80) ? 'pkt' : 'logpage';

      dlSession.push({
        recNum: rocketRecNum,
        snr: logSnr === 0x7F ? wsSnr : logSnr / 4,
        rssi: wsRssi,
        ts: logTs,           // rocket's ms-since-boot
        payload: payload,
        type: recType,
        srcDev: srcDev
      });
      added++;
    }

    // Track LoRa DL speed
    if (added > 0) {
      if (dlSpeedStartMs === 0) dlSpeedStartMs = Date.now();
      dlSpeedRecs += added;
      dlSpeedBytes += buf.byteLength;
      var dlElapsed = (Date.now() - dlSpeedStartMs) / 1000;
      if (dlElapsed > 0.1) {
        var dlRps = (dlSpeedRecs / dlElapsed).toFixed(0);
        var dlKbps = (dlSpeedBytes / 1024 / dlElapsed).toFixed(1);
        document.getElementById('fst').textContent = 'DL: '+dlSession.length+'recs '+dlRps+'r/s '+dlKbps+'kB/s';
      }
    }
    // Log to packet log
    var el = document.getElementById('log');
    var e = document.createElement('div');
    e.className = 'le';
    e.style.color = '#a8f';
    e.textContent = 'DL 0xCA dev:' + srcDev + ' start#' + startIdx + ' x' + recCount + ' decoded:' + added + ' total:' + dlSession.length;
    el.insertBefore(e, el.firstChild);
    while (el.children.length > 1000) el.removeChild(el.lastChild);

    // Rebuild sessions to include the download data
    rebuildSessions();
    updateNav();

    // If viewing the DL session, live-append the new records to charts
    if (viewIdx >= 0 && sessions[viewIdx] === dlSession && charts) {
      for (var j = dlSession.length - added; j < dlSession.length; j++) {
        var r = dlSession[j];
        showRecord(r, true, false);
      }
    }
  }

  // Show a single record — dispatches to showPkt for 0xAF telem or showLogPage for raw pages
  function showRecord(rec, toCharts, isLive) {
    if (rec.type === 'logpage') {
      showLogPage(rec.payload, rec.snr, rec.ts, toCharts, isLive);
    } else {
      showPkt(rec.payload, rec.snr, rec.rssi || -130, rec.ts, toCharts, isLive);
    }
  }

  // Show a raw data page from a log record (not wrapped in 0xAF telemetry)
  function showLogPage(payload, snr, bootMs, toC, isLive) {
    var bytes = new Uint8Array(payload);
    if (bytes.length < 2) return;
    var pageType = bytes[0];
    var df = PD[pageType];
    if (!df) return;
    if (bytes.length < 1 + df.s) return;
    var dv = new DataView(payload);
    // Thrust curve (0x0E): pass totalLen for variable-length decode; load chart instead of pushing
    if (pageType === 14) {
      var d14 = df.d(dv, 1, bytes.length - 1);
      if (df.f) setV(pageType, df.f(d14), isLive);
      updateDetail(pageType, d14);
      loadThrustCurve(d14);
      return;
    }
    var d = df.d(dv, 1); // decode starting at offset 1 (after page type byte)
    if (df.f) setV(pageType, df.f(d), isLive);
    updateDetail(pageType, d);
    if (!toC || !charts) return;

    // Push to charts — same logic as showPkt but without the 0xAF header
    var baroAlt = null, gpsAlt = null, bgRssi = null;
    if (pageType === 2) baroAlt = d.alt_cm / 100;
    if (pageType === 6) gpsAlt = d.alt_cm / 100;
    if (pageType === 12) bgRssi = d.bgRssi;
    if (pageType === 3) pushChart(charts.mag, bootMs, [d.x, d.y, d.z]);
    if (pageType === 4) pushChart(charts.acc, bootMs, [d.x, d.y, d.z]);
    if (pageType === 5) pushChart(charts.gyr, bootMs, [d.x, d.y, d.z]);
    if (pageType === 8) { pushChart(charts.batt, bootMs, [d.batt, null]); updateRktBatt(d.batt); }
    // For altitude chart: only push if this page contributes altitude data
    if (pageType === 2 || pageType === 6) {
      pushChart(charts.alt, bootMs, [null, gpsAlt, baroAlt]);
    }
    if (pageType === 12) {
      pushChart(charts.snr, bootMs, [null, null, bgRssi]);
    }
  }

  var lastStatusPollMs = 0;
  var statusPollInterval = null;

  function startStatusPolling() {
    if (statusPollInterval) return;  // Already polling
    statusPollInterval = setInterval(function() {
      if (bleConnected && bleStatusChar_) {
        bleReadStatus().catch(function(e) { console.error('Auto-poll BLE status failed:', e); });
      } else if (rktBleConnected && rktStatusChar_) {
        rktReadStatus().catch(function(e) { console.error('Auto-poll rocket status failed:', e); });
      }
    }, 10000);  // Poll every 10 seconds
  }

  function stopStatusPolling() {
    if (statusPollInterval) {
      clearInterval(statusPollInterval);
      statusPollInterval = null;
    }
  }

  function fetchStatus(cb){
    // BLE path: read status characteristic
    if (bleConnected && bleStatusChar_) {
      bleReadStatus().then(function() { if (cb) cb(null); });
      return;
    }
    // HTTP path (WiFi)
    var x=new XMLHttpRequest();x.open('GET',getBaseHttp()+'/api/status');x.onload=function(){if(x.status===200){try{var s=JSON.parse(x.responseText);serverUptimeMs=s.uptimeMs;serverSyncClockMs=Date.now();document.getElementById('val-logrec').textContent=s.records;if(typeof s.baseBattMv==='number'){var be=document.getElementById('val-basebatt');be.textContent=s.baseBattMv+'mV';be.style.color=s.baseBattMv>3500?'#0f0':s.baseBattMv>3300?'#ff0':'#f44';if(charts&&charts.batt){var bootMs=serverUptimeMs;pushChart(charts.batt,bootMs,[null,s.baseBattMv])}}updateNonceFromStatus(s);if(cb)cb(s)}catch(e){}}};x.send()
  }

  // Rebuild sessions from fetched logs + LoRa download session.
  // Fetched sessions split on timestamp resets (reboot boundaries).
  // All record types included (0xAF telem packets and raw data pages).
  // The LoRa download session is appended as a single session if non-empty.
  function rebuildSessions(){
    var recs=[];
    for(var k in allFetched)recs.push(allFetched[k]);
    recs.sort(function(a,b){return a.recNum-b.recNum});
    sessions=[];
    var cur=[];
    for(var i=0;i<recs.length;i++){
      var r=recs[i];
      // Assign type if not already set
      if(!r.type){
        var b0=r.payload.byteLength>0?new Uint8Array(r.payload)[0]:0;
        r.type=(b0===0xAF)?'pkt':'logpage';
      }
      // New session on timestamp reset (reboot boundary)
      if(cur.length>0&&r.ts<cur[cur.length-1].ts){sessions.push(cur);cur=[];}
      cur.push(r);
    }
    if(cur.length>0)sessions.push(cur);
    // Append LoRa download session if non-empty
    if (dlSession.length > 0) sessions.push(dlSession);
  }
  function updateFetchProgress(cursor) {
    var elapsed = (Date.now() - fetchSpeedStartMs) / 1000;
    if (elapsed < 0.1) return;
    var rps = (fetchSpeedRecs / elapsed).toFixed(0);
    var kbps = (fetchSpeedBytes / 1024 / elapsed).toFixed(1);
    var total = 0; for (var k in allFetched) total++;
    var progress = (cursor > 0) ? ' #'+cursor+'↓' : '';
    document.getElementById('fst').textContent = total+'recs '+rps+'r/s '+kbps+'kB/s'+progress;
  }
  function fetchIntermediateUpdate(cursor) {
    rebuildSessions();
    updateNav();
    // Don't re-render chart on every batch — too slow as allFetched grows.
    // Final showView is called in the Done handler.
    updateFetchProgress(cursor);
  }
  function loadHistory(){
    fetchAbort=false;
    fetchAutoNav=true;
    fetchSpeedStartMs=Date.now();
    fetchSpeedRecs=0;
    fetchSpeedBytes=0;
    document.getElementById('btn-stop').style.display='inline';

    // Rocket direct BLE fetch path
    if (rktBleConnected && rktFetchChar_) {
      rktReadStatus().then(function() {
        var newest = parseInt(document.getElementById('val-rktlogrec').textContent) || 0;
        if (newest === 0) { document.getElementById('btn-stop').style.display='none'; return; }
        var cursor = newest;
        var batchSize = 2000;
        function rktPage() {
          if (fetchAbort || cursor <= 0) { rktDone(); return; }
          var start = Math.max(cursor - batchSize, 0);
          var count = cursor - start;
          if (start >= fetchedLowest && cursor <= fetchedHighest) {
            cursor = start; rktPage(); return;
          }
          document.getElementById('fst').textContent = '#'+start+'..'+(start+count-1)+'…';
          rktFetchDone = false;
          rktFetchLogs(start, count, function() { updateFetchProgress(cursor); }).then(function(ok) {
            if (!ok) { rktDone(); return; }  // timeout/abort — stop instead of skipping ahead
            cursor = start;
            fetchIntermediateUpdate(cursor);
            if (cursor > 0 && !fetchAbort) rktPage(); else rktDone();
          });
        }
        function rktDone() {
          document.getElementById('btn-stop').style.display='none';
          rebuildSessions();
          var total=0; for(var k in allFetched) total++;
          var elapsed=(Date.now()-fetchSpeedStartMs)/1000;
          var rps=elapsed>0.1?(fetchSpeedRecs/elapsed).toFixed(0):'0';
          var kbps=elapsed>0.1?(fetchSpeedBytes/1024/elapsed).toFixed(1):'0';
          document.getElementById('fst').textContent=total+'recs '+sessions.length+'sess | '+rps+'r/s '+kbps+'kB/s';
          updateNav();
          if(sessions.length>0) showView(sessions.length-1);
        }
        rktPage();
      });
      return;
    }

    // Base station BLE fetch path
    if (bleConnected && bleLogFetchChar_) {
      bleReadStatus().then(function() {
        var recEl = document.getElementById('val-logrec');
        var newest = parseInt(recEl.textContent) || 0;
        if (newest === 0) { document.getElementById('btn-stop').style.display='none'; return; }
        // Fetch newest-first, skip already-fetched ranges, so re-clicking extends downward
        var cursor = newest;
        var batchSize = 2000;
        function blePage() {
          if (fetchAbort || cursor <= 0) { bleDone(); return; }
          var start = Math.max(cursor - batchSize, 0);
          var count = cursor - start;
          // Skip ranges already in memory (allows re-click to continue where we left off)
          if (start >= fetchedLowest && cursor <= fetchedHighest) {
            cursor = start; blePage(); return;
          }
          document.getElementById('fst').textContent = '#'+start+'..'+(start+count-1)+'…';
          bleFetchLogs(start, count, function() { updateFetchProgress(cursor); }).then(function(ok) {
            if (!ok) { bleDone(); return; }  // timeout/abort — stop instead of skipping ahead
            cursor = start;
            fetchIntermediateUpdate(cursor);
            if (cursor > 0 && !fetchAbort) blePage(); else bleDone();
          });
        }
        function bleDone() {
          document.getElementById('btn-stop').style.display='none';
          rebuildSessions();
          var total=0; for(var k in allFetched) total++;
          var elapsed=(Date.now()-fetchSpeedStartMs)/1000;
          var rps=elapsed>0.1?(fetchSpeedRecs/elapsed).toFixed(0):'0';
          var kbps=elapsed>0.1?(fetchSpeedBytes/1024/elapsed).toFixed(1):'0';
          document.getElementById('fst').textContent=total+'recs '+sessions.length+'sess | '+rps+'r/s '+kbps+'kB/s';
          updateNav();
          if(sessions.length>0) showView(sessions.length-1);
        }
        blePage();
      });
      return;
    }

    // HTTP path (WiFi) — fetches newest-first; re-clicking continues from oldest unvisited record
    fetchStatus(function(st){
      var oldest=st?(st.oldest||0):0, newest=st?(st.records||0):0;
      if(newest<=oldest){document.getElementById('fst').textContent='No logs';document.getElementById('btn-stop').style.display='none';return;}
      var cursor=newest;
      function page(){
        if(fetchAbort||cursor<=oldest){done();return;}
        var start=Math.max(cursor-200,oldest),count=cursor-start;
        if(start>=fetchedLowest&&cursor<=fetchedHighest){cursor=start;if(cursor>oldest)page();else done();return;}
        document.getElementById('fst').textContent='#'+start+'..'+(start+count-1)+'…';
        var x=new XMLHttpRequest();
        x.open('GET',getBaseHttp()+'/api/logs?start='+start+'&count='+count);
        x.responseType='arraybuffer';
        x.onload=function(){
          if(x.status===200&&x.response&&x.response.byteLength>0){
            fetchSpeedBytes+=x.response.byteLength;
            var buf=x.response,dv=new DataView(buf),off=0;
            while(off+10<=dv.byteLength){
              var rn=dv.getUint32(off,true);off+=4;
              var pl=dv.getUint8(off);off++;
              var sn=dv.getInt8(off);off++;
              var ts=dv.getUint32(off,true);off+=4;
              if(pl===0||off+pl>dv.byteLength)break;
              if(!allFetched[rn]){var p=buf.slice(off,off+pl);var b0=pl>0?new Uint8Array(p)[0]:0;allFetched[rn]={recNum:rn,snr:sn/4,ts:ts,payload:p,type:b0===0xAF?'pkt':'logpage'};if(rn<fetchedLowest)fetchedLowest=rn;if(rn>fetchedHighest)fetchedHighest=rn;fetchSpeedRecs++;}
              off+=pl;
            }
          }
          cursor=start;
          fetchIntermediateUpdate(cursor);
          if(cursor>oldest&&!fetchAbort)page();else done();
        };
        x.onerror=function(){done();};
        x.send();
      }
      function done(){
        document.getElementById('btn-stop').style.display='none';
        rebuildSessions();
        var total=0;for(var k in allFetched)total++;
        var elapsed=(Date.now()-fetchSpeedStartMs)/1000;
        var rps=elapsed>0.1?(fetchSpeedRecs/elapsed).toFixed(0):'0';
        var kbps=elapsed>0.1?(fetchSpeedBytes/1024/elapsed).toFixed(1):'0';
        document.getElementById('fst').textContent=total+'recs '+sessions.length+'sess | '+rps+'r/s '+kbps+'kB/s';
        updateNav();if(sessions.length>0)showView(sessions.length-1);
      }
      page();
    });
  }

  var wsObj = null;
  function connectWS(){try{wsObj=new WebSocket(getBaseWs());wsObj.binaryType='arraybuffer'}catch(e){setWs(false);return}wsObj.onopen=function(){setWs(true);fetchStatus()};wsObj.onclose=function(){setWs(false)};wsObj.onmessage=function(ev){if(!(ev.data instanceof ArrayBuffer)||ev.data.byteLength<13)return;var dv=new DataView(ev.data);var pktBuf=ev.data.slice(12);var snr=dv.getFloat32(0,true);var rssi=dv.getFloat32(4,true);var recNum=dv.getInt32(8,true);var firstByte=new Uint8Array(pktBuf)[0];if(firstByte===0xCA){onLogChunk(pktBuf,snr,rssi,recNum)}else{onLivePkt(pktBuf,snr,rssi,recNum)}};wsObj.onerror=function(){wsObj.close()}}

  // =============================================================
  // BLE TRANSPORT (Web Bluetooth GATT)
  // =============================================================
  // Connects directly to the base station (or rocket) via BLE.
  // Uses the same binary formats as WebSocket/HTTP — no protocol changes.
  // BLE allows the phone to keep its mobile internet connection.

  var BLE_SERVICE_UUID       = '4d4f4f4e-5348-4f54-4253-000000000000';
  var BLE_TELEM_CHAR_UUID    = '4d4f4f4e-5348-4f54-4253-000000000001';
  var BLE_CMD_CHAR_UUID      = '4d4f4f4e-5348-4f54-4253-000000000002';
  var BLE_STATUS_CHAR_UUID   = '4d4f4f4e-5348-4f54-4253-000000000003';
  var BLE_LOGFETCH_CHAR_UUID = '4d4f4f4e-5348-4f54-4253-000000000004';
  var BLE_OTA_CHAR_UUID      = '4d4f4f4e-5348-4f54-4253-000000000006';

  var bleDevice = null;
  var bleServer_ = null;
  var bleTelemChar_ = null;
  var bleCmdChar_ = null;
  var bleStatusChar_ = null;
  var bleLogFetchChar_ = null;
  var bleOtaChar_ = null;
  var bleConnected = false;

  // BLE log fetch state for sliding window flow control
  var bleFetchQueue = [];     // pending fetch requests [{start,count,resolve}]
  var bleFetchBuffer = [];    // received records from current fetch batch
  var bleFetchDone = false;

  function setBle(on) {
    document.getElementById('dot-ble').className = 'dot ' + (on ? 'dg' : 'dd');
    document.getElementById('val-ble').textContent = on ? 'connected' : 'off';
    document.getElementById('btn-ble').textContent = on ? 'Disconnect BLE' : 'Connect BLE';
    document.getElementById('btn-ble').style.background = on ? '#533' : '#335';
    document.getElementById('btn-ble').style.color = on ? '#faa' : '#8af';
  }

  async function connectBLE() {
    if (bleConnected) {
      // Disconnect
      if (bleDevice && bleDevice.gatt && bleDevice.gatt.connected) {
        bleDevice.gatt.disconnect();
      }
      return;
    }

    if (!navigator.bluetooth) {
      alert('Web Bluetooth is not available. Use Chrome/Edge on HTTPS or localhost.');
      return;
    }

    try {
      bleDevice = await navigator.bluetooth.requestDevice({
        filters: [
          { services: [BLE_SERVICE_UUID] },
          { namePrefix: 'Wifi' }
        ],
        optionalServices: [BLE_SERVICE_UUID]
      });

      bleDevice.addEventListener('gattserverdisconnected', function() {
        bleConnected = false;
        setBle(false);
        bleTelemChar_ = null; bleCmdChar_ = null; bleStatusChar_ = null; bleLogFetchChar_ = null;
        stopStatusPolling();
        console.log('BLE disconnected');
      });

      setBle(false);
      document.getElementById('val-ble').textContent = 'connecting...';

      bleServer_ = await bleDevice.gatt.connect();
      // Request max MTU — Web Bluetooth handles this automatically on most platforms
      var svc = await bleServer_.getPrimaryService(BLE_SERVICE_UUID);

      bleTelemChar_ = await svc.getCharacteristic(BLE_TELEM_CHAR_UUID);
      bleCmdChar_ = await svc.getCharacteristic(BLE_CMD_CHAR_UUID);
      bleStatusChar_ = await svc.getCharacteristic(BLE_STATUS_CHAR_UUID);
      bleLogFetchChar_ = await svc.getCharacteristic(BLE_LOGFETCH_CHAR_UUID);

      // Subscribe to telemetry notifications
      await bleTelemChar_.startNotifications();
      bleTelemChar_.addEventListener('characteristicvaluechanged', function(ev) {
        var buf = ev.target.value.buffer;
        if (buf.byteLength < 13) return;
        var dv = new DataView(buf);
        var pktBuf = buf.slice(12);
        var snr = dv.getFloat32(0, true);
        var rssi = dv.getFloat32(4, true);
        var recNum = dv.getInt32(8, true);
        var firstByte = new Uint8Array(pktBuf)[0];
        if (firstByte === 0xCA) {
          onLogChunk(pktBuf, snr, rssi, recNum);
        } else {
          onLivePkt(pktBuf, snr, rssi, recNum);
        }
      });

      // OTA characteristic (optional — may not exist on older firmware)
      try {
        bleOtaChar_ = await svc.getCharacteristic(BLE_OTA_CHAR_UUID);
        await bleOtaChar_.startNotifications();
        bleOtaChar_.addEventListener('characteristicvaluechanged', onOtaNotify);
      } catch(e) { bleOtaChar_ = null; }

      // Subscribe to log fetch notifications
      await bleLogFetchChar_.startNotifications();
      bleLogFetchChar_.addEventListener('characteristicvaluechanged', function(ev) {
        var buf = ev.target.value.buffer;
        if (buf.byteLength === 0) {
          // End marker — current batch done
          bleFetchDone = true;
          return;
        }
        // Track bytes for speed display
        fetchSpeedBytes += buf.byteLength;
        // Parse records — same format as /api/logs response
        var dv = new DataView(buf);
        var off = 0;
        while (off + 10 <= buf.byteLength) {
          var rn = dv.getUint32(off, true); off += 4;
          var pl = dv.getUint8(off); off++;
          var sn = dv.getInt8(off); off++;
          var ts = dv.getUint32(off, true); off += 4;
          if (pl === 0 || off + pl > buf.byteLength) break;
          var payload = buf.slice(off, off + pl);
          off += pl;
          if (!allFetched[rn]) {
            var b0 = payload.byteLength > 0 ? new Uint8Array(payload)[0] : 0;
            allFetched[rn] = { recNum: rn, snr: sn / 4, ts: ts, payload: payload, type: b0 === 0xAF ? 'pkt' : 'logpage' };
            if (rn < fetchedLowest) fetchedLowest = rn;
            if (rn > fetchedHighest) fetchedHighest = rn;
            fetchSpeedRecs++;
          }
        }
      });

      bleConnected = true;
      setBle(true);

      // Fetch initial status via BLE
      bleReadStatus();
      startStatusPolling();

      console.log('BLE connected, MTU negotiated by OS');

    } catch (err) {
      console.error('BLE connect failed:', err);
      setBle(false);
      document.getElementById('val-ble').textContent = err.message || 'failed';
    }
  }

  // Update rocket battery display from any source (status poll or page 0x08 telemetry)
  function updateRktBatt(battMv) {
    var be = document.getElementById('val-rktbatt');
    be.textContent = battMv + 'mV';
    be.style.color = battMv > 3500 ? '#0f0' : battMv > 3300 ? '#ff0' : '#f44';
  }

  // Update nonce field from either base or rocket status
  function updateNonceFromStatus(s) {
    if (typeof s.nonce === 'number') {
      var ne = document.getElementById('cmd-nonce');
      var srvNext = s.nonce + 1;
      var cur = parseInt(ne.value) || 0;
      if (srvNext > cur) { ne.value = srvNext; try { localStorage.setItem('cmd-nonce', '' + srvNext); } catch(e) {} }
    }
  }

  // Read status via BLE (equivalent to GET /api/status)
  async function bleReadStatus() {
    if (!bleConnected || !bleStatusChar_) return;
    try {
      var val = await bleStatusChar_.readValue();
      var dec = new TextDecoder();
      var json = dec.decode(val.buffer);
      var s = JSON.parse(json);
      serverUptimeMs = s.uptimeMs;
      serverSyncClockMs = Date.now();
      document.getElementById('val-logrec').textContent = s.records;
      if (typeof s.baseBattMv === 'number') {
        var be = document.getElementById('val-basebatt');
        be.textContent = s.baseBattMv + 'mV';
        be.style.color = s.baseBattMv > 3500 ? '#0f0' : s.baseBattMv > 3300 ? '#ff0' : '#f44';
      }
      updateNonceFromStatus(s);
    } catch (e) {
      console.error('BLE status read failed:', e);
    }
  }

  // Send command via BLE (equivalent to POST /api/command)
  async function bleSendCommand(postBody) {
    if (!bleConnected || !bleCmdChar_) return { ok: false, msg: 'BLE not connected' };
    try {
      await bleCmdChar_.writeValueWithResponse(postBody);
      // Read back result byte
      var result = await bleCmdChar_.readValue();
      var code = result.getUint8(0);
      if (code === 0x00) return { ok: true, msg: 'OK' };
      return { ok: false, msg: 'error code ' + code };
    } catch (e) {
      return { ok: false, msg: e.message || 'BLE write failed' };
    }
  }

  // Fetch logs via BLE (equivalent to GET /api/logs)
  // Uses sliding window: request a batch, wait for notifications, request more.
  // onProgress (optional): called every ~400ms during wait so the UI can update mid-batch.
  async function bleFetchLogs(startRec, count, onProgress) {
    if (!bleConnected || !bleLogFetchChar_) return false;
    bleFetchDone = false;
    var req = new Uint8Array(6);
    var rdv = new DataView(req.buffer);
    rdv.setUint32(0, startRec, true);
    rdv.setUint16(4, count, true);
    try {
      await bleLogFetchChar_.writeValueWithResponse(req);
      // Wait for end marker (empty notification), calling onProgress periodically
      var timeout = 10000;
      var start = Date.now();
      var lastProg = Date.now();
      while (!bleFetchDone && !fetchAbort && (Date.now() - start) < timeout) {
        await new Promise(function(r) { setTimeout(r, 50); });
        if (onProgress && Date.now() - lastProg > 400) {
          lastProg = Date.now();
          onProgress();
        }
      }
      return bleFetchDone;
    } catch (e) {
      console.error('BLE fetch error:', e);
      return false;
    }
  }

  // =============================================================
  // ROCKET DIRECT BLE (Web Bluetooth — connects directly to rocket)
  // Different service UUIDs from base station. No SNR/RSSI wrapper.
  // Telem packets are sequences of [len][data] records (data[0] = type).
  // Type 0xAF = telemetry header record; other types = data pages.
  // =============================================================

  var RKT_SERVICE_UUID       = '524f434b-4554-5354-424c-000000000000';
  var RKT_TELEM_CHAR_UUID    = '524f434b-4554-5354-424c-000000000001';
  var RKT_CMD_CHAR_UUID      = '524f434b-4554-5354-424c-000000000002';
  var RKT_STATUS_CHAR_UUID   = '524f434b-4554-5354-424c-000000000003';
  var RKT_CONNSET_CHAR_UUID  = '524f434b-4554-5354-424c-000000000004';
  var RKT_LOGFETCH_CHAR_UUID = '524f434b-4554-5354-424c-000000000005';
  var RKT_OTA_CHAR_UUID      = '524f434b-4554-5354-424c-000000000006';

  // ConnSet command type bytes
  var RKT_CS_INTERVAL = 0x01;
  var RKT_CS_PAGEMASK = 0x02;
  var RKT_CS_PHY      = 0x03;

  var rktDevice = null;
  var rktServer_ = null;
  var rktTelemChar_ = null;
  var rktCmdChar_ = null;
  var rktStatusChar_ = null;
  var rktConnSetChar_ = null;
  var rktFetchChar_ = null;
  var rktOtaChar_ = null;
  var rktBleConnected = false;
  var rktFetchDone = false;
  var rktFetchBuffer = [];

  function setRocketBle(on) {
    document.getElementById('dot-rktble').className = 'dot ' + (on ? 'dg' : 'dd');
    document.getElementById('val-rktble').textContent = on ? 'connected' : 'off';
    document.getElementById('btn-rktble').textContent = on ? 'Disconnect Rkt' : 'Rkt BLE';
    document.getElementById('btn-rktble').style.background = on ? '#533' : '#353';
    document.getElementById('btn-rktble').style.color = on ? '#faa' : '#8fa';
  }

  // Cached last header record from rocket BLE (ArrayBuffer starting with 0xAF)
  var rktLastHeader = null;

  // Parse a BLE telem notification from the rocket.
  // PDU format: [captureUs u64 LE][records...]
  // Each record: [len u8][data: len bytes], data[0]=type.
  // Types 0x00-0x7F = data pages; 0xAF = header record.
  // captureUs is the rocket's micros() at capture time — used as x-axis (ms).
  // All records in one PDU share the same capture timestamp.
  function onRktTelemNotify(buf) {
    if (buf.byteLength < 9) return;  // need at least 8-byte ts + 1 record byte
    var dv = new DataView(buf);
    // Read capture timestamp as two u32 halves (JS has no u64)
    var tsLo = dv.getUint32(0, true);
    var tsHi = dv.getUint32(4, true);
    // Convert microseconds to milliseconds for chart x-axis
    var captureMs = (tsHi * 4294967.296 + tsLo / 1000);
    var off = 8;
    while (off + 1 < buf.byteLength) {
      var recLen = dv.getUint8(off);
      off++;
      if (recLen === 0 || off + recLen > buf.byteLength) break;
      var recData = buf.slice(off, off + recLen);
      off += recLen;
      var type = new Uint8Array(recData)[0];
      if (type === 0xAF) {
        rktLastHeader = recData;
        var rec = {recNum: -1, snr: 127, rssi: -130, ts: captureMs, payload: recData, type: 'pkt'};
        rktLiveRecs.push(rec);
        if (rktLiveRecs.length > MAX_PTS) rktLiveRecs.shift();
        showPkt(recData, 127, -130, captureMs, viewIdx === -1 && liveSource === 'rkt', true);
        if (viewIdx === -1 && liveSource === 'rkt') addLogRec(rec, false);
      } else {
        var rec = {recNum: -1, snr: 127, rssi: -130, ts: captureMs, payload: recData, type: 'logpage'};
        rktLiveRecs.push(rec);
        if (rktLiveRecs.length > MAX_PTS) rktLiveRecs.shift();
        showLogPage(recData, 127, captureMs, viewIdx === -1 && liveSource === 'rkt', true);
        if (viewIdx === -1 && liveSource === 'rkt') addLogRec(rec, false);
      }
    }
  }

  async function rktReadStatus() {
    if (!rktBleConnected || !rktStatusChar_) return;
    try {
      var val = await rktStatusChar_.readValue();
      var dec = new TextDecoder();
      var json = dec.decode(val.buffer);
      var s = JSON.parse(json);
      if (typeof s.logIdx === 'number') {
        document.getElementById('val-rktlogrec').textContent = s.logIdx;
      }
      if (typeof s.batt === 'number') { updateRktBatt(s.batt); }
      // Sync rocket boot clock for correct x-axis on charts
      if (typeof s.uptime === 'number') {
        rktUptimeMs = s.uptime;
        rktSyncClockMs = Date.now();
      }
      updateNonceFromStatus(s);
    } catch (e) {
      console.error('Rkt BLE status read failed:', e);
    }
  }

  // Send command via rocket BLE
  async function rktBleSendCommand(postBody) {
    if (!rktBleConnected || !rktCmdChar_) return { ok: false, msg: 'Rocket BLE not connected' };
    try {
      await rktCmdChar_.writeValueWithResponse(postBody);
      return { ok: true, msg: 'OK' };
    } catch (e) {
      return { ok: false, msg: e.message || 'Rkt BLE write failed' };
    }
  }

  // Write ConnSet to rocket: set subscription interval in microseconds
  async function rktSetInterval(intervalUs) {
    if (!rktBleConnected || !rktConnSetChar_) return;
    var buf = new Uint8Array(5);
    buf[0] = RKT_CS_INTERVAL;
    var dv = new DataView(buf.buffer);
    dv.setUint32(1, intervalUs, true);
    try { await rktConnSetChar_.writeValueWithoutResponse(buf); } catch (e) {}
  }

  // Write ConnSet to rocket: set PHY (0=1M, 1=2M, 2=Coded-S2, 3=Coded-S8)
  async function rktSetPhy(phy) {
    if (!rktBleConnected || !rktConnSetChar_) return;
    var buf = new Uint8Array(2);
    buf[0] = RKT_CS_PHY;
    buf[1] = phy;
    try { await rktConnSetChar_.writeValueWithoutResponse(buf); } catch (e) {}
  }

  // Fetch logs directly from rocket flash
  async function rktFetchLogs(startRec, count, onProgress) {
    if (!rktBleConnected || !rktFetchChar_) return false;
    rktFetchDone = false;
    var req = new Uint8Array(6);
    var rdv = new DataView(req.buffer);
    rdv.setUint32(0, startRec, true);
    rdv.setUint16(4, count, true);
    try {
      await rktFetchChar_.writeValueWithResponse(req);
      var timeout = 30000;
      var start = Date.now();
      var lastProg = Date.now();
      while (!rktFetchDone && !fetchAbort && (Date.now() - start) < timeout) {
        await new Promise(function(r) { setTimeout(r, 50); });
        if (onProgress && Date.now() - lastProg > 400) {
          lastProg = Date.now();
          onProgress();
        }
      }
      return rktFetchDone;
    } catch (e) {
      console.error('Rkt BLE fetch error:', e);
      return false;
    }
  }

  async function connectRocketBLE() {
    if (rktBleConnected) {
      if (rktDevice && rktDevice.gatt && rktDevice.gatt.connected) {
        rktDevice.gatt.disconnect();
      }
      return;
    }

    if (!navigator.bluetooth) {
      alert('Web Bluetooth not available. Use Chrome/Edge on HTTPS or localhost.');
      return;
    }

    try {
      rktDevice = await navigator.bluetooth.requestDevice({
        filters: [
          { services: [RKT_SERVICE_UUID] },
          { namePrefix: 'Moonshot-Rocket' }
        ],
        optionalServices: [RKT_SERVICE_UUID]
      });

      rktDevice.addEventListener('gattserverdisconnected', function() {
        rktBleConnected = false;
        setRocketBle(false);
        rktTelemChar_ = null; rktCmdChar_ = null;
        rktStatusChar_ = null; rktConnSetChar_ = null; rktFetchChar_ = null;
        stopStatusPolling();
        console.log('Rocket BLE disconnected');
      });

      setRocketBle(false);
      document.getElementById('val-rktble').textContent = 'connecting...';

      rktServer_ = await rktDevice.gatt.connect();
      var svc = await rktServer_.getPrimaryService(RKT_SERVICE_UUID);

      rktTelemChar_   = await svc.getCharacteristic(RKT_TELEM_CHAR_UUID);
      rktCmdChar_     = await svc.getCharacteristic(RKT_CMD_CHAR_UUID);
      rktStatusChar_  = await svc.getCharacteristic(RKT_STATUS_CHAR_UUID);
      rktConnSetChar_ = await svc.getCharacteristic(RKT_CONNSET_CHAR_UUID);
      rktFetchChar_   = await svc.getCharacteristic(RKT_LOGFETCH_CHAR_UUID);

      // Subscribe to telemetry notifications
      await rktTelemChar_.startNotifications();
      rktTelemChar_.addEventListener('characteristicvaluechanged', function(ev) {
        onRktTelemNotify(ev.target.value.buffer);
      });

      // OTA characteristic (optional — may not exist on older firmware)
      try {
        rktOtaChar_ = await svc.getCharacteristic(RKT_OTA_CHAR_UUID);
        await rktOtaChar_.startNotifications();
        rktOtaChar_.addEventListener('characteristicvaluechanged', onOtaNotify);
      } catch(e) { rktOtaChar_ = null; }

      // Subscribe to log fetch notifications
      await rktFetchChar_.startNotifications();
      rktFetchChar_.addEventListener('characteristicvaluechanged', function(ev) {
        var buf = ev.target.value.buffer;
        if (buf.byteLength === 0) {
          rktFetchDone = true;
          return;
        }
        // Parse [recNum u32 LE][len u8][snr i8][ts u32 LE][payload: len bytes]
        // Same format as base station log fetch — store directly into allFetched.
        fetchSpeedBytes += buf.byteLength;
        var dv = new DataView(buf);
        var off = 0;
        while (off + 10 <= buf.byteLength) {
          var rn = dv.getUint32(off, true); off += 4;
          var pl = dv.getUint8(off); off++;
          var sn = dv.getInt8(off); off++;
          var ts = dv.getUint32(off, true); off += 4;
          if (pl === 0 || off + pl > buf.byteLength) break;
          var payload = buf.slice(off, off + pl);
          off += pl;
          if (!allFetched[rn]) {
            var b0 = payload.byteLength > 0 ? new Uint8Array(payload)[0] : 0;
            allFetched[rn] = { recNum: rn, snr: sn / 4, ts: ts, payload: payload, type: b0 === 0xAF ? 'pkt' : 'logpage' };
            if (rn < fetchedLowest) fetchedLowest = rn;
            if (rn > fetchedHighest) fetchedHighest = rn;
            fetchSpeedRecs++;
          }
        }
      });

      rktBleConnected = true;
      setRocketBle(true);

      // Auto-switch live view to rocket
      liveSource = 'rkt';
      showView(-1);

      // Read initial status
      rktReadStatus();
      startStatusPolling();

      console.log('Rocket BLE connected');

    } catch (err) {
      console.error('Rocket BLE connect failed:', err);
      setRocketBle(false);
      document.getElementById('val-rktble').textContent = err.message || 'failed';
    }
  }

  // NOTE: This WiFi AP has no internet. CDN scripts only work if the browser
  // has them cached from a previous session on a different network. Always
  // provide a fallback and a manual "load from cache" button.
  var CDN_URLS = [
    'https://cdn.jsdelivr.net/npm/chart.js@4',
    'https://cdn.jsdelivr.net/npm/hammerjs@2',
    'https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@2'
  ];
  var cdnLoaded = 0, cdnTotal = CDN_URLS.length;

  function loadCDN() {
    var btn = document.getElementById('btn-cdn');
    if (typeof Chart !== 'undefined' && !charts) {
      initCharts();
      rebuildChartsWithZoom();
      btn.textContent = 'Charts OK';
      return;
    }
    cdnLoaded = 0;
    btn.textContent = 'Loading 0/' + cdnTotal + '...';

    function loadNext(i) {
      if (i >= CDN_URLS.length) {
        if (typeof Chart !== 'undefined') {
          initCharts();
          rebuildChartsWithZoom();
          btn.textContent = 'Charts OK';
        } else {
          btn.textContent = 'Charts (retry)';
        }
        return;
      }
      var s = document.createElement('script');
      s.src = CDN_URLS[i];
      s.onload = function() {
        cdnLoaded++;
        btn.textContent = 'Loading ' + cdnLoaded + '/' + cdnTotal + '...';
        loadNext(i + 1);
      };
      s.onerror = function() {
        btn.textContent = 'CDN fail (' + (i + 1) + '/' + cdnTotal + ') retry';
      };
      document.head.appendChild(s);
    }
    loadNext(0);
  }
  var chartSyncing = false;
  function syncChartsFrom(src){
    if(chartSyncing||!charts)return;
    chartSyncing=true;
    var xMin=src.scales.x.min,xMax=src.scales.x.max;
    for(var k in charts){var c=charts[k];if(c!==src){c.options.scales.x.min=xMin;c.options.scales.x.max=xMax;c.update('none')}}
    chartSyncing=false;
  }
  function rebuildChartsWithZoom(){if(!charts)return;for(var k in charts){var c=charts[k];if(c.options&&c.options.plugins){c.options.plugins.zoom={zoom:{wheel:{enabled:true},pinch:{enabled:true},mode:'x',onZoom:function(ctx){syncChartsFrom(ctx.chart)}},pan:{enabled:true,mode:'x',onPan:function(ctx){syncChartsFrom(ctx.chart)}}};c.update()}}}

  // =============================================================
  // VOICE CALLOUTS (Browser TTS — Web Speech API)
  // =============================================================

  var voiceEnabled = false;
  var voiceMode = 'normal'; // 'normal' or 'moonshot'
  var voiceQueue = [];
  var voiceBusy = false;
  var voiceLastPopMs = 0;
  var VOICE_POP_INTERVAL = 4000; // ms between queue pops as fallback

  // State tracking — reset when we say "FTS is safe"
  var vSaidArmed = false;
  var vSaidReadyLast = 0; // timestamp of last "ready to launch" callout
  var vSaidGroundTestLast = 0; // timestamp of last "ground test" callout
  var vSaidBoost = false;
  var vSaidCoast = false;
  var vSaidApogee = false;
  var vSaidLanded = false;
  var vSaidSafe = false; // tracks whether we need to say disarmed/safe
  var vSaidLowBattLast = 0; // timestamp of last low battery callout

  function voiceReset() {
    vSaidArmed = false;
    vSaidReadyLast = 0;
    vSaidGroundTestLast = 0;
    vSaidBoost = false;
    vSaidCoast = false;
    vSaidApogee = false;
    vSaidLanded = false;
    vSaidSafe = false;
    vSaidLowBattLast = 0;
  }

  function abbreviateAlt(m) {
    // Abbreviated number for TTS — see voice_callouts spec
    if (m < 0) return 'minus ' + abbreviateAlt(-m);
    m = Math.round(m);
    if (m <= 20) return '' + m;
    if (m <= 99) return '' + (Math.floor(m / 10) * 10);
    if (m <= 9999) return Math.round(m / 100) + ' hundred';
    return '' + m;
  }

  function voiceSay(text) {
    voiceQueue.push(text);
    voicePump();
  }

  function voicePump() {
    if (!voiceEnabled) { voiceQueue = []; return; }
    if (voiceBusy) return;
    if (voiceQueue.length === 0) return;
    var now = Date.now();
    // Respect minimum interval between pops
    if (now - voiceLastPopMs < 800) return;
    var text = voiceQueue.shift();
    voiceBusy = true;
    voiceLastPopMs = now;
    var sts = document.getElementById('voice-status');
    if (sts) sts.textContent = text;
    try {
      var utt = new SpeechSynthesisUtterance(text);
      utt.rate = 1.2;
      // Prefer Google UK English Female if available
      var voices = speechSynthesis.getVoices();
      for (var vi = 0; vi < voices.length; vi++) {
        if (voices[vi].name === 'Google UK English Female') { utt.voice = voices[vi]; break; }
      }
      utt.onend = function() { voiceBusy = false; voicePump(); };
      utt.onerror = function() { voiceBusy = false; voicePump(); };
      speechSynthesis.speak(utt);
      // Fallback: if onend doesn't fire, force pop after interval
      setTimeout(function() {
        if (voiceBusy) { voiceBusy = false; voicePump(); }
      }, VOICE_POP_INTERVAL);
    } catch(e) {
      voiceBusy = false;
    }
  }

  // Called from showPkt for every live telemetry packet
  // p = decoded telem, ph = flight phase, arm = armed flag, fusAltM = fusion alt in meters or null
  function voiceOnTelem(p, ph, arm, fusAltM) {
    var moon = (voiceMode === 'moonshot');

    // "Armed" — first time armed flag is set or phase=pad_ready(1) or phase=ground_test(9)
    if ((arm || ph === 1 || ph === 9) && !vSaidArmed) {
      vSaidArmed = true;
      vSaidSafe = false; // now we need to watch for disarm
      voiceSay(moon ? 'Flight Termination System is armed' : 'Armed');
    }

    // "Ready to launch" — phase=pad_ready, every 30s
    if (ph === 1 && vSaidArmed) {
      var now = Date.now();
      if (now - vSaidReadyLast > 30000) {
        vSaidReadyLast = now;
        voiceSay(moon ? 'Moonshot is ready to launch' : 'Ready to launch');
      }
    }

    // "Ground test" / "Static fire" — phase=ground_test(9), every 30s
    if (ph === 9 && vSaidArmed) {
      var now3 = Date.now();
      if (now3 - vSaidGroundTestLast > 30000) {
        vSaidGroundTestLast = now3;
        voiceSay(moon ? 'Static fire' : 'Ground test');
      }
    }

    // "Boost" — first time phase=boost(2)
    if (ph === 2 && !vSaidBoost) {
      vSaidBoost = true;
      voiceSay(moon ? 'Max Q' : 'Boost');
    }

    // "Coast" — first time phase=coast(3)
    if (ph === 3 && !vSaidCoast) {
      vSaidCoast = true;
      voiceSay(moon ? 'Stage 2 ignition' : 'Coast');
    }

    // "Apogee at X meters" — triggered by peaks page (0x09) in certain conditions
    // Checked via the data page in the packet
    if (p.pg && p.pg.t === 9 && !vSaidApogee) {
      var peakAltCm = p.pg.d.alt; // cm MSL
      var peakAltM = Math.round(peakAltCm / 100);
      // Condition 1: peaks page during flight state 4-8
      // Condition 2: peaks page with peak >= 30m above current fusion alt while armed
      var triggerByPhase = (ph >= 4 && ph <= 8);
      var triggerByDiff = arm && fusAltM !== null && (peakAltCm / 100 - fusAltM >= 30);
      if (triggerByPhase || triggerByDiff) {
        vSaidApogee = true;
        voiceSay(moon ? 'Nominal orbit ' + peakAltM + ' kilometers' : 'Apogee at ' + peakAltM + ' meters');
      }
    }

    // Descent altitude callouts — after apogee said, phase 5-7, only if not already speaking
    if (vSaidApogee && ph >= 5 && ph <= 7 && fusAltM !== null) {
      if (!voiceBusy && voiceQueue.length === 0) {
        var abbr = abbreviateAlt(fusAltM);
        voiceSay(moon ? 'TLI burn ' + abbr + ' seconds' : abbr + ' meters');
      }
    }

    // "Landed" — first time phase=landed(8)
    if (ph === 8 && !vSaidLanded) {
      vSaidLanded = true;
      voiceSay(moon ? 'Trans-lunar injection complete' : 'Landed');
    }

    // "Low Battery" / "Solar Panel Deployment Failed" — low batt flag or system health batt<3300, every 60s
    var lowBatt = !!(p.flags & 0x100); // bit 8 = low battery
    if (p.pg && p.pg.t === 8 && p.pg.d.batt < 3300) lowBatt = true;
    if (lowBatt) {
      var now2 = Date.now();
      if (now2 - vSaidLowBattLast > 60000) {
        vSaidLowBattLast = now2;
        voiceSay(moon ? 'Solar Panel Deployment Failed' : 'Low Battery');
      }
    }

    // "Disarmed" / "FTS is safe" — first packet with armed=false after having said armed
    if (!arm && vSaidArmed && !vSaidSafe) {
      vSaidSafe = true;
      voiceSay(moon ? 'FTS is safe' : 'Disarmed');
      // Reset all flags for next flight
      voiceReset();
      vSaidSafe = true; // keep safe flag so we don't re-trigger until next arm
    }
  }

  // =============================================================
  // COMMANDS
  // =============================================================

  var derivedKeyBytes = null; // Uint8Array(32)

  var CMD_HELP = {
    '0x01': 'ARM: Arm rocket (13-byte params). test_mode flag enters GROUND_TEST phase instead of PAD_READY. Sets log protection 60s back.',
    '0x02': 'DISARM: Return to idle. Refused if pyro/chute active.',
    '0x03': 'FIRE PYRO/CHUTE: Energise channel for N ms (RMT hardware-timed). Requires armed. Transitions to GROUND_TEST unless stay_in_phase flag set.',
    '0x10': 'SET TX RATE: +N=Hz (1-127), -N=sec between, 0=stop telemetry.',
    '0x12': 'SET ROCKET RADIO: Channel, SF, TX power. Stored to NVS.',
    '0x30': 'SET RELAY RADIO: Primary + backhaul settings. Stored to NVS.',
    '0x31': 'ENABLE WIFI: Start WiFi AP on target.',
    '0x32': 'DISABLE WIFI: Stop WiFi AP. Re-enables on power cycle.',
    '0x20': 'LOG DOWNLOAD: Request records from target on download radio settings.',
    '0x40': 'PING: Request ack page. Link test.',
    '0x41': 'SET SYNC: Establish slot clock sync point. No parameters.',
    '0xF0': 'REBOOT: Software restart. Refused if armed (rocket).',
    '0xF1': 'LOG ERASE: Clear log ring buffer. Refused if armed.',
    '0x50': 'OTA BEGIN: Open firmware update session. Erases inactive OTA partition (~2s). Refused while armed or rollback pending.',
    '0x52': 'OTA CONFIRM: After rebooting into new firmware, confirm it is good. Cancels rollback. If never sent, next reboot reverts to previous firmware.'
  };

  function onCmdChange() {
    var val = document.getElementById('cmd-select').value;
    var panels = document.querySelectorAll('.cmd-params');
    for (var i = 0; i < panels.length; i++) panels[i].className = 'cmd-params';
    if (val) {
      var panel = document.getElementById('params-' + val);
      if (panel) panel.className = 'cmd-params visible';
      var sel = document.getElementById('cmd-select');
      var group = sel.options[sel.selectedIndex].parentNode.label;
      if (group === 'Rocket') document.getElementById('cmd-target').value = 146;
      else if (group === 'Base/Relay') document.getElementById('cmd-target').value = 157;
      // 'Any device' commands leave target unchanged
    }
    document.getElementById('cmd-help-text').textContent = CMD_HELP[val] || '';
  }

  function deriveKey() {
    var password = document.getElementById('cmd-pass').value;
    if (!password) {
      document.getElementById('cmd-key-status').textContent = 'Enter a password first';
      return;
    }
    var enc = new TextEncoder();
    var passBytes = enc.encode(password);
    var saltBytes = enc.encode('Moonshot-HMAC-SALT');


    document.getElementById('cmd-key-status').textContent = 'Deriving (100k iterations)...';

    // Use setTimeout to not freeze UI during PBKDF2
    setTimeout(function() {
      derivedKeyBytes = pbkdf2Sha256(passBytes, saltBytes, 100000, 32);
      document.getElementById('cmd-key-status').textContent =
        'Key: ' + toHex(derivedKeyBytes).substring(0, 1) + '... (32 bytes)';
      otaUpdateHmac();
    }, 50);
  }

  function copyKey() {
    if (!derivedKeyBytes) {
      document.getElementById('cmd-key-status').textContent = 'Derive a key first';
      return;
    }
    var hex = toHex(derivedKeyBytes);
    // Clipboard API needs HTTPS — fallback to textarea selection for HTTP
    if (navigator.clipboard && window.isSecureContext) {
      navigator.clipboard.writeText(hex).then(function(){
        document.getElementById('cmd-key-status').textContent = 'Copied!';
      });
    } else {
      var ta = document.createElement('textarea');
      ta.value = hex; ta.style.position = 'fixed'; ta.style.left = '-9999px';
      document.body.appendChild(ta); ta.select();
      try { document.execCommand('copy'); document.getElementById('cmd-key-status').textContent = 'Copied!'; }
      catch(e) { document.getElementById('cmd-key-status').textContent = 'Key: ' + hex; }
      document.body.removeChild(ta);
    }
  }

  function buildParams(cmdId) {
    var buf = [];
    switch (cmdId) {
      case 0x01: {
        // ARM — always 13-byte format
        var boostG  = parseInt(document.getElementById('p01-boostG').value)  || 3000;
        var boostA  = parseInt(document.getElementById('p01-boostAlt').value) || 100;
        var coastA  = parseInt(document.getElementById('p01-coastAlt').value) || 200;
        var mainA   = parseInt(document.getElementById('p01-mainAlt').value)  || 100;
        var forceArm= document.getElementById('p01-force').checked  ? 1 : 0;
        var testMode= document.getElementById('p01-test').checked   ? 2 : 0;
        var flags   = forceArm | testMode;
        var drogMs  = parseInt(document.getElementById('p01-drogMs').value)   || 3000;
        var mainMs  = parseInt(document.getElementById('p01-mainMs').value)   || 3000;
        buf.push(boostG & 0xFF, (boostG >> 8) & 0xFF);
        buf.push(boostA & 0xFF, (boostA >> 8) & 0xFF);
        buf.push(coastA & 0xFF, (coastA >> 8) & 0xFF);
        buf.push(mainA  & 0xFF, (mainA  >> 8) & 0xFF);
        buf.push(flags & 0xFF);
        buf.push(drogMs & 0xFF, (drogMs >> 8) & 0xFF);
        buf.push(mainMs & 0xFF, (mainMs >> 8) & 0xFF);
        break;
      }
      case 0x03: {
        buf.push(parseInt(document.getElementById('p03-ch').value) & 0xFF);
        var dur = parseInt(document.getElementById('p03-dur').value) & 0xFFFF;
        buf.push(dur & 0xFF, (dur >> 8) & 0xFF);
        var stay = document.getElementById('p03-stay').checked ? 1 : 0;
        buf.push(stay & 0xFF);
        break;
      }
      case 0x10:
        buf.push(parseInt(document.getElementById('p10-rate').value) & 0xFF);
        break;
      case 0x12:
        buf.push(parseInt(document.getElementById('p12-ch').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p12-sf').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p12-pwr').value) & 0xFF);
        break;
      case 0x30:
        buf.push(parseInt(document.getElementById('p30-pch').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p30-psf').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p30-ppwr').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p30-bch').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p30-bsf').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p30-bpwr').value) & 0xFF);
        break;
      case 0x20:
        var st = parseInt(document.getElementById('p20-start').value);
        buf.push(st & 0xFF, (st >> 8) & 0xFF, (st >> 16) & 0xFF, (st >> 24) & 0xFF);
        var cnt = parseInt(document.getElementById('p20-count').value) & 0xFFFF;
        buf.push(cnt & 0xFF, (cnt >> 8) & 0xFF);
        buf.push(parseInt(document.getElementById('p20-ch').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p20-sf').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p20-bw').value) & 0xFF);
        buf.push(parseInt(document.getElementById('p20-pwr').value) & 0xFF);
        break;
      case 0x41:
        // SET SYNC: no parameters
        break;
    }
    return buf;
  }

  function sendCommand() {
    var result = document.getElementById('cmd-result');
    if (!derivedKeyBytes) { result.textContent = 'ERROR: Derive a key first'; result.style.color = '#f44'; return; }

    var cmdVal = document.getElementById('cmd-select').value;
    if (!cmdVal) { result.textContent = 'ERROR: Select a command'; result.style.color = '#f44'; return; }

    var cmdId = parseInt(cmdVal);
    var targetId = parseInt(document.getElementById('cmd-target').value) & 0xFF;
    var nonce = parseInt(document.getElementById('cmd-nonce').value) >>> 0;
    var waitMs = parseInt(document.getElementById('cmd-wait').value);
    var sends = parseInt(document.getElementById('cmd-sends').value);
    var params = buildParams(cmdId);

    var pkt = [];
    pkt.push(0x9A);
    pkt.push(targetId);
    pkt.push(cmdId);
    pkt.push(nonce & 0xFF, (nonce >> 8) & 0xFF, (nonce >> 16) & 0xFF, (nonce >> 24) & 0xFF);
    for (var i = 0; i < params.length; i++) pkt.push(params[i]);

    var hmacInput = new Uint8Array(pkt.length);
    for (var i = 0; i < pkt.length; i++) hmacInput[i] = pkt[i];

    var hmacFull = hmacSha256(derivedKeyBytes, hmacInput);
    var hmac10 = hmacFull.slice(0, 10);

    var fullPkt = new Uint8Array(pkt.length + 10);
    for (var i = 0; i < pkt.length; i++) fullPkt[i] = pkt[i];
    for (var i = 0; i < 10; i++) fullPkt[pkt.length + i] = hmac10[i];

    result.style.color = '#0f0';
    result.textContent = 'Packet: [' + toHex(fullPkt) + '] (' + fullPkt.length + 'B)\n' +
      'HMAC in: [' + toHex(hmacInput) + ']\n' +
      'Sending (wait=' + waitMs + 'ms, sends=' + sends + ')...';

    // Build the transport body: [waitMs LE][sends][pktLen][packet bytes]
    var postBody = new Uint8Array(4 + fullPkt.length);
    var dv = new DataView(postBody.buffer);
    dv.setUint16(0, waitMs, true);
    dv.setUint8(2, sends);
    dv.setUint8(3, fullPkt.length);
    postBody.set(fullPkt, 4);

    // Route command based on selected transport
    var transport = (document.getElementById('cmd-transport') || {}).value || 'auto';
    var useRktBle   = (transport === 'rktble') && rktBleConnected;
    var useBaseBle  = (transport === 'baseBle' || (transport === 'auto' && bleConnected)) && bleConnected;
    var useHttp     = !useRktBle && !useBaseBle;

    if (useRktBle) {
      rktBleSendCommand(postBody).then(function(r) {
        result.textContent += '\nRkt BLE: ' + (r.ok ? 'OK' : r.msg);
        if (!r.ok) result.style.color = '#f80';
      });
    } else if (useBaseBle) {
      bleSendCommand(postBody).then(function(r) {
        result.textContent += '\nBase BLE: ' + (r.ok ? 'OK' : r.msg);
        if (!r.ok) result.style.color = '#f80';
      });
    } else {
      var xhr = new XMLHttpRequest();
      xhr.open('POST', getBaseHttp() + '/api/command');
      xhr.setRequestHeader('Content-Type', 'application/octet-stream');
      xhr.onload = function() {
        if (xhr.status === 200) {
          result.textContent += '\nServer: OK';
        } else {
          result.textContent += '\nServer: HTTP ' + xhr.status;
          result.style.color = '#f80';
        }
      };
      xhr.onerror = function() {
        result.textContent += '\nServer: not reachable';
        result.style.color = '#f80';
      };
      xhr.send(postBody);
    }

    // Auto-increment nonce
    var newNonce = nonce + 1;
    document.getElementById('cmd-nonce').value = newNonce;
    try { localStorage.setItem('cmd-nonce', newNonce); } catch(e) {}

    if (cmdId === 0x20) {
      var dlStart = parseInt(document.getElementById('p20-start').value) || 0;
      var dlCount = parseInt(document.getElementById('p20-count').value) || 100;
      var dirDown = document.getElementById('p20-dir-down').checked;
      document.getElementById('p20-start').value = dirDown ? Math.max(0, dlStart - dlCount) : dlStart + dlCount;
      // Reset DL speed counters so rate shown reflects this download batch
      dlSpeedStartMs = 0; dlSpeedRecs = 0; dlSpeedBytes = 0;
    }
  }

  // =============================================================
  // OTA FIRMWARE UPDATE
  // =============================================================

  var otaFileBytes = null;   // Uint8Array of selected .bin
  var otaFileHmac  = null;   // Uint8Array(32) HMAC of entire firmware
  var otaRunning   = false;

  // Status byte → human-readable string
  var OTA_STATUS_MSG = {
    0x00: 'OK',
    0x01: 'not activated (send OTA BEGIN first)',
    0x02: 'write/begin/erase failed',
    0x03: 'offset gap',
    0x04: 'HMAC mismatch — image rejected',
    0x05: 'esp_ota_end failed',
    0x06: 'currently verifying, chunks blocked',
    0x07: 'refused (armed or rollback pending)',
    0x08: 'chunk crosses 512-byte sector boundary'
  };

  function onOtaNotify(ev) {
    var data = new Uint8Array(ev.target.value.buffer);
    var prog = document.getElementById('ota-progress');
    if (!prog) return;
    if (data.length >= 5 && data[0] === 0xA0) {
      // Progress report: [0xA0][bytesWritten u32 LE]
      var bw = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
      prog.textContent += '\nDevice progress: ' + bw + ' bytes written';
    } else if (data.length >= 1) {
      var code = data[0];
      var msg = OTA_STATUS_MSG[code] !== undefined ? OTA_STATUS_MSG[code] : ('code 0x' + code.toString(16));
      prog.textContent += '\nDevice: ' + msg;
      if (code !== 0x00 && otaRunning) {
        // Non-zero on the OTA char while uploading — stop and report
        prog.textContent += ' — upload aborted';
        otaRunning = false;
        document.getElementById('ota-send').disabled = false;
        document.getElementById('ota-send').textContent = 'FLASH FIRMWARE';
      }
    }
    prog.scrollTop = prog.scrollHeight;
  }

  function otaUpdateHmac() {
    if (!otaFileBytes) return;
    var el = document.getElementById('ota-hmac-status');
    if (!el) return;
    if (!derivedKeyBytes) { el.textContent = 'HMAC: derive key first'; return; }
    setTimeout(function() {
      otaFileHmac = hmacSha256(derivedKeyBytes, otaFileBytes);
      el.textContent = 'HMAC: ' + toHex(otaFileHmac).substring(0, 16) + '... (32 bytes)';
    }, 10);
  }

  async function sendOtaFirmware() {
    if (otaRunning) return;
    var prog = document.getElementById('ota-progress');
    prog.textContent = '';

    if (!derivedKeyBytes) { prog.textContent = 'ERROR: derive key first'; return; }
    if (!otaFileBytes)    { prog.textContent = 'ERROR: select a .bin file'; return; }

    var target = document.getElementById('ota-target').value;
    var otaChar = (target === 'rkt') ? rktOtaChar_ : bleOtaChar_;
    var cmdChar = (target === 'rkt') ? rktCmdChar_ : bleCmdChar_;
    var useHttp = (target === 'base') && !bleOtaChar_ && !bleConnected;

    if (!useHttp && !otaChar) { prog.textContent = 'ERROR: OTA char not available — connect BLE first'; return; }
    if (!useHttp && !cmdChar) { prog.textContent = 'ERROR: command char not available'; return; }

    // Recompute HMAC just before sending
    otaFileHmac = hmacSha256(derivedKeyBytes, otaFileBytes);
    document.getElementById('ota-hmac-status').textContent =
      'HMAC: ' + toHex(otaFileHmac).substring(0, 16) + '...';

    otaRunning = true;
    document.getElementById('ota-send').disabled = true;
    document.getElementById('ota-send').textContent = 'UPLOADING...';

    var targetId = parseInt(document.getElementById('ota-target-id').value) & 0xFF;
    var nonce = parseInt(document.getElementById('cmd-nonce').value) >>> 0;

    // Helper: build and send a no-param 0x9A command, return result byte (BLE) or throw (HTTP)
    async function sendCmd(cmdId) {
      var pktArr = [0x9A, targetId, cmdId];
      pktArr.push(nonce & 0xFF, (nonce >> 8) & 0xFF, (nonce >> 16) & 0xFF, (nonce >> 24) & 0xFF);
      var hmacInput = new Uint8Array(pktArr);
      var mac10 = hmacSha256(derivedKeyBytes, hmacInput).slice(0, 10);
      var fullPkt = new Uint8Array(pktArr.length + 10);
      fullPkt.set(hmacInput); fullPkt.set(mac10, pktArr.length);
      nonce++;
      document.getElementById('cmd-nonce').value = nonce;
      try { localStorage.setItem('cmd-nonce', nonce); } catch(e2) {}

      // Both BLE and HTTP use the [waitMs u16 LE][sends u8][pktLen u8][packet] wrapper.
      var body = new Uint8Array(4 + fullPkt.length);
      var bdv = new DataView(body.buffer);
      bdv.setUint16(0, 3000, true); bdv.setUint8(2, 1); bdv.setUint8(3, fullPkt.length);
      body.set(fullPkt, 4);
      if (useHttp) {
        var r = await fetch(getBaseHttp() + '/api/command', {
          method: 'POST', headers: {'Content-Type': 'application/octet-stream'}, body: body
        });
        if (!r.ok) throw new Error('CMD 0x' + cmdId.toString(16) + ' HTTP ' + r.status);
      } else {
        await cmdChar.writeValueWithResponse(body);
      }
    }

    // Send CMD_OTA_BEGIN and wait for 0x00 notify on the OTA char.
    // The device notifies 0x00 after erase + esp_ota_begin completes (~1-2s).
    prog.textContent = 'Sending OTA BEGIN (erasing partition, ~2s)...';
    var beginReady = new Promise(function(resolve, reject) {
      var timeout = setTimeout(function() {
        reject(new Error('OTA BEGIN timeout — no ready signal from device'));
      }, 10000);
      function onBeginNotify(ev) {
        var d = new Uint8Array(ev.target.value.buffer);
        if (d.length >= 1 && d[0] === 0x00) {
          clearTimeout(timeout);
          otaChar.removeEventListener('characteristicvaluechanged', onBeginNotify);
          resolve();
        } else if (d.length >= 1 && d[0] !== 0xA0) {
          // Error during begin
          clearTimeout(timeout);
          otaChar.removeEventListener('characteristicvaluechanged', onBeginNotify);
          var msg = OTA_STATUS_MSG[d[0]] || ('code 0x' + d[0].toString(16));
          reject(new Error('OTA BEGIN failed: ' + msg));
        }
      }
      otaChar.addEventListener('characteristicvaluechanged', onBeginNotify);
    });
    await sendCmd(0x50);
    await beginReady;

    var CHUNK = 508;  // 4-byte offset prefix + 508 bytes data = 512-byte BLE write limit
    var totalBytes = otaFileBytes.length;
    var totalChunks = Math.ceil(totalBytes / CHUNK);
    prog.textContent = 'Sending ' + totalChunks + ' chunks (' + totalBytes + ' bytes)...';

    try {
      for (var i = 0; i < totalChunks; i++) {
        if (!otaRunning) break;  // aborted by error notify
        var offset = i * CHUNK;
        var end = Math.min(offset + CHUNK, totalBytes);
        var chunk = otaFileBytes.slice(offset, end);

        var pkt = new Uint8Array(4 + chunk.length);
        var dv = new DataView(pkt.buffer);
        dv.setUint32(0, offset, true);
        pkt.set(chunk, 4);

        if (useHttp) {
          var resp = await fetch(getBaseHttp() + '/api/ota/chunk', {
            method: 'PUT', body: pkt
          });
          if (!resp.ok) throw new Error('HTTP ' + resp.status);
          var statusByte = new Uint8Array(await resp.arrayBuffer())[0];
          if (statusByte !== 0x00) {
            var msg = OTA_STATUS_MSG[statusByte] || ('0x' + statusByte.toString(16));
            throw new Error('Device error: ' + msg);
          }
        } else {
          await otaChar.writeValueWithoutResponse(pkt);
        }

        // Update progress every 32 chunks and yield to keep UI responsive
        if (i % 32 === 31 || i === totalChunks - 1) {
          prog.textContent = 'Chunk ' + (i + 1) + '/' + totalChunks +
            ' (' + Math.round((i + 1) / totalChunks * 100) + '%)';
          await new Promise(function(r) { setTimeout(r, 20); });
        }
      }

      if (!otaRunning) return;  // aborted

      prog.textContent += '\nAll chunks sent. Sending finalize...';

      // Build CMD_OTA_FINALIZE (0x51) packet: [0x9A][target][0x51][nonce u32][fwSize u32][fwHmac 32][HMAC10]
      var pktArr = [0x9A, targetId, 0x51];
      pktArr.push(nonce & 0xFF, (nonce >> 8) & 0xFF, (nonce >> 16) & 0xFF, (nonce >> 24) & 0xFF);
      pktArr.push(totalBytes & 0xFF, (totalBytes >> 8) & 0xFF,
                  (totalBytes >> 16) & 0xFF, (totalBytes >> 24) & 0xFF);
      for (var j = 0; j < 32; j++) pktArr.push(otaFileHmac[j]);
      var hmacInput = new Uint8Array(pktArr);
      var cmdHmac10 = hmacSha256(derivedKeyBytes, hmacInput).slice(0, 10);
      var fullPkt = new Uint8Array(pktArr.length + 10);
      fullPkt.set(hmacInput);
      fullPkt.set(cmdHmac10, pktArr.length);
      nonce++;
      document.getElementById('cmd-nonce').value = nonce;
      try { localStorage.setItem('cmd-nonce', nonce); } catch(e2) {}

      var finalBody = new Uint8Array(4 + fullPkt.length);
      var fbdv = new DataView(finalBody.buffer);
      fbdv.setUint16(0, 2000, true); fbdv.setUint8(2, 1); fbdv.setUint8(3, fullPkt.length);
      finalBody.set(fullPkt, 4);
      if (useHttp) {
        var r = await fetch(getBaseHttp() + '/api/command', {
          method: 'POST', headers: {'Content-Type': 'application/octet-stream'}, body: finalBody
        });
        if (!r.ok) throw new Error('Finalize HTTP ' + r.status + ': ' + await r.text());
      } else {
        await cmdChar.writeValueWithResponse(finalBody);
      }

      prog.textContent += '\nFinalize sent. Device rebooting...\n' +
        'Reconnect and send 0x52 OTA CONFIRM to keep new firmware,\n' +
        'or reboot without confirming to roll back.';

    } catch(err) {
      prog.textContent += '\nERROR: ' + (err.message || err);
    } finally {
      otaRunning = false;
      document.getElementById('ota-send').disabled = false;
      document.getElementById('ota-send').textContent = 'FLASH FIRMWARE';
    }
  }

  // =============================================================
  // MAP — RANGE TESTING VISUALIZER
  // =============================================================
  var mapPts = [];           // live-session map points
  var mapUserGps = null;     // {lat,lon,acc} from watchPosition
  var mapGpsWid = null;
  var mapAckNonces = {};     // dedup cmdack by nonce
  var mapLastWall = 0;       // wall clock of last live packet
  var mapVOx = 0, mapVOy = 0, mapVScale = 0; // view: pan offsets + scale (0=needs fit)
  var mapRefLat = 0, mapRefLon = 0, mapRefCos = 1; // projection reference
  var mapT0 = 0, mapT1 = 1; // timeline filter 0..1
  var mapShowSNR = true;
  var mapShowRkt = true, mapShowUsr = true, mapShowLin = false;
  var mapShowMis = true, mapShowAck = true;
  var mapVisible = false;
  var mapHits = [];          // [{x,y,pt}] hit-test targets rebuilt each draw
  var mapDragMoved = false;
  var mapPointers = {}, mapPinchD0 = 0, mapPinchCx = 0, mapPinchCy = 0;
  var mapDevHeading = null;  // degrees clockwise from north (null = no fix)
  var mapAnimInt = null;
  var mapIDB = null;
  var mapRafPend = false;
  var mapAgeMax = 600;       // max age of WIN_LR points on map (seconds)

  function mapScheduleDraw() {
    if (!mapVisible || mapRafPend) return;
    mapRafPend = true;
    requestAnimationFrame(function() { mapRafPend = false; if (mapVisible) mapDraw(); });
  }

  // GPS watch — started/stopped with map panel open/close
  function mapStartGps() {
    if (!navigator.geolocation) { mapGpsStatus('N/A'); return; }
    if (mapGpsWid !== null) return;
    mapGpsWid = navigator.geolocation.watchPosition(
      function(p) {
        mapUserGps = {lat: p.coords.latitude, lon: p.coords.longitude, acc: p.coords.accuracy || 0};
        var e = document.getElementById('map-gps-st');
        if (e) { e.textContent = 'GPS \u00b1' + mapUserGps.acc.toFixed(0) + 'm'; e.style.color = mapUserGps.acc < 15 ? '#0f0' : mapUserGps.acc < 50 ? '#ff0' : '#f80'; }
      },
      function(err) { mapUserGps = null; mapGpsStatus(err.code === 1 ? 'denied' : 'err'); },
      {enableHighAccuracy: true, maximumAge: 2000, timeout: 15000}
    );
  }
  function mapStopGps() {
    if (mapGpsWid !== null && navigator.geolocation) { navigator.geolocation.clearWatch(mapGpsWid); mapGpsWid = null; }
  }
  function mapGpsStatus(s) { var e = document.getElementById('map-gps-st'); if (e) e.textContent = 'GPS: ' + s; }
  function mapStartCompass() {
    if (!window.DeviceOrientationEvent) return;
    var listen = function() {
      window.addEventListener('deviceorientationabsolute', mapOnDevOrientation, true);
      window.addEventListener('deviceorientation', mapOnDevOrientation, true);
    };
    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
      DeviceOrientationEvent.requestPermission().then(function(s) {
        if (s === 'granted') listen();
        else { var b = document.getElementById('map-cmp-btn'); if (b) b.style.display = ''; }
      }).catch(function() { var b = document.getElementById('map-cmp-btn'); if (b) b.style.display = ''; });
    } else { listen(); }
  }
  function mapStopCompass() {
    window.removeEventListener('deviceorientationabsolute', mapOnDevOrientation, true);
    window.removeEventListener('deviceorientation', mapOnDevOrientation, true);
    mapDevHeading = null;
  }
  function mapOnDevOrientation(e) {
    if (e.absolute && e.alpha !== null) { mapDevHeading = (360 - e.alpha + 360) % 360; return; }
    if (typeof e.webkitCompassHeading === 'number') { mapDevHeading = e.webkitCompassHeading; return; }
    if (e.alpha !== null) mapDevHeading = (360 - e.alpha + 360) % 360;
  }

  // Color helpers
  function mapSigColor(snr, rssi) {
    var t = mapShowSNR ? Math.max(0, Math.min(1, (snr + 20) / 30)) : Math.max(0, Math.min(1, (rssi + 130) / 70));
    var r = t < 0.5 ? 255 : Math.round(255 * (2 - 2*t));
    var g = t < 0.5 ? Math.round(255 * 2*t) : 255;
    return 'rgb(' + r + ',' + g + ',0)';
  }
  function mapAltColor(alt, minA, maxA) {
    if (alt === null || alt === undefined) return '#777';
    var t = maxA > minA ? Math.max(0, Math.min(1, (alt - minA) / (maxA - minA))) : 0.5;
    var r, g, b;
    if (t < 0.25)      { r = 0; g = Math.round(255*t*4); b = 255; }
    else if (t < 0.5)  { r = 0; g = 255; b = Math.round(255*(2 - t*4)); }
    else if (t < 0.75) { r = Math.round(255*(t*4 - 2)); g = 255; b = 0; }
    else               { r = 255; g = Math.round(255*(4 - t*4)); b = 0; }
    return 'rgb(' + r + ',' + g + ',' + b + ')';
  }

  // Add a live telemetry packet to the map.
  // Must be called AFTER showPkt so fullGpsLat/fullGpsLon are already updated.
  function mapAddPt(buf, snr, rssi, recNum) {
    var p = decodeTelem(buf);
    if (!p) return;
    var rktG = resolveGps(p.latF, p.lonF);
    // If this packet had page 0x01, use the higher-precision coords directly
    if (p.pg && p.pg.t === 1) { rktG = {lat: p.pg.d.lat, lon: p.pg.d.lon}; }
    var pgC = p.pg ? {t: p.pg.t, d: Object.assign({}, p.pg.d)} : null;
    var pt = {
      wallTime: lastPktMs, type: 'telem',
      uLat: mapUserGps ? mapUserGps.lat : null,
      uLon: mapUserGps ? mapUserGps.lon : null,
      uAcc: mapUserGps ? mapUserGps.acc : null,
      rLat: rktG ? rktG.lat : null, rLon: rktG ? rktG.lon : null,
      alt: p.altM !== -32768 ? p.altM : null,
      snr: snr, rssi: rssi, phase: p.flags & 0xF,
      recNum: recNum, pg: pgC, isMissed: false
    };
    mapPts.push(pt);
    mapLastWall = lastPktMs;
    // CmdAck page (0x0A): emit one dedicated point per unique nonce
    if (p.pg && p.pg.t === 10 && !mapAckNonces[p.pg.d.nonce]) {
      mapAckNonces[p.pg.d.nonce] = true;
      mapPts.push({wallTime: lastPktMs, type: 'cmdack',
        uLat: pt.uLat, uLon: pt.uLon, uAcc: pt.uAcc,
        rLat: pt.rLat, rLon: pt.rLon, alt: pt.alt,
        snr: p.pg.d.snr, rssi: p.pg.d.rssi, phase: pt.phase,
        recNum: recNum, pg: pgC, isMissed: false,
        ackNonce: p.pg.d.nonce, ackRes: p.pg.d.res});
    }
    mapScheduleDraw();
  }

  // Missing-packet timer: fires every second.
  // Reads p10-rate input directly; only acts for rates giving period >= 2s.
  setInterval(function() {
    if (!mapLastWall || !mapUserGps) return;
    var rateEl = document.getElementById('p10-rate');
    var rate = rateEl ? (parseInt(rateEl.value)|0) : 0;
    if (rate === 0) return; // TX stopped — no markers
    var period = rate > 0 ? 1 : Math.max(1, -rate); // fast rates use 1s period
    if ((Date.now() - mapLastWall) / 1000 < period + 0.9) return;
    // Don't spam: skip if we just added a missed marker within this period
    for (var i = mapPts.length - 1; i >= 0; i--) {
      if (!mapPts[i].isMissed) break;
      if ((Date.now() - mapPts[i].wallTime) / 1000 < period * 0.9) return;
    }
    mapPts.push({wallTime: Date.now(), type: 'missed',
      uLat: mapUserGps.lat, uLon: mapUserGps.lon, uAcc: mapUserGps.acc,
      rLat: null, rLon: null, alt: null, snr: null, rssi: null,
      phase: null, recNum: null, pg: null, isMissed: true});
    mapLastWall = Date.now();
    mapScheduleDraw();
  }, 1000);

  // Projection: equirectangular with cos(lat) correction
  function mapFitView(cw, ch) {
    var pts = mapGetFiltered();
    var lats = [], lons = [];
    pts.forEach(function(p) {
      if (p.uLat !== null) { lats.push(p.uLat); lons.push(p.uLon); }
      if (p.rLat !== null) { lats.push(p.rLat); lons.push(p.rLon); }
    });
    if (!lats.length) return;
    var la0 = Math.min.apply(null,lats), la1 = Math.max.apply(null,lats);
    var lo0 = Math.min.apply(null,lons), lo1 = Math.max.apply(null,lons);
    mapRefLat = (la0+la1)/2; mapRefLon = (lo0+lo1)/2;
    mapRefCos = Math.cos(mapRefLat * Math.PI / 180);
    var dLa = la1-la0 || 0.0005, dLo = (lo1-lo0)*mapRefCos || 0.0005;
    mapVScale = Math.max(10, Math.min((cw*0.82)/dLo, (ch*0.82)/dLa));
    mapVOx = 0; mapVOy = 0;
  }
  function mapW2C(lat, lon, cw, ch) {
    return {
      x: cw/2 + mapVOx + (lon - mapRefLon) * mapRefCos * mapVScale,
      y: ch/2 + mapVOy - (lat - mapRefLat) * mapVScale
    };
  }

  // Timeline filter
  function mapGetFiltered() {
    if (!mapPts.length) return [];
    var t0 = mapPts[0].wallTime, t1 = mapPts[mapPts.length-1].wallTime, rng = t1-t0 || 1;
    var lo = t0 + rng*mapT0, hi = t0 + rng*mapT1;
    return mapPts.filter(function(p) { return p.wallTime >= lo && p.wallTime <= hi; });
  }
  function mapUpdateTimeLbls() {
    if (!mapPts.length) {
      document.getElementById('map-t0l').textContent = '--';
      document.getElementById('map-t1l').textContent = '--';
      document.getElementById('map-cntl').textContent = '';
      return;
    }
    var t0 = mapPts[0].wallTime, t1 = mapPts[mapPts.length-1].wallTime, rng = t1-t0 || 1;
    function fmtT(ms) { var d = new Date(ms); return pad2(d.getHours())+':'+pad2(d.getMinutes())+':'+pad2(d.getSeconds()); }
    document.getElementById('map-t0l').textContent = fmtT(t0+rng*mapT0);
    document.getElementById('map-t1l').textContent = fmtT(t0+rng*mapT1);
    document.getElementById('map-cntl').textContent = mapGetFiltered().length + '/' + mapPts.length + 'pts';
  }

  // Main draw
  function mapDraw() {
    var canvas = document.getElementById('map-canvas');
    if (!canvas) return;
    var dpr = window.devicePixelRatio || 1;
    var cw = canvas.clientWidth || 300, ch = canvas.clientHeight || 400;
    if (canvas.width !== Math.round(cw*dpr) || canvas.height !== Math.round(ch*dpr)) {
      canvas.width = Math.round(cw*dpr); canvas.height = Math.round(ch*dpr);
      mapVScale = 0; // force re-fit after resize
    }
    var ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#0d0d0d'; ctx.fillRect(0, 0, cw, ch);
    mapHits = [];

    var pts = mapGetFiltered();
    if (!pts.length) {
      ctx.fillStyle = '#444'; ctx.font = '12px Courier New'; ctx.textAlign = 'center';
      ctx.fillText(mapPts.length ? 'No points in range' : 'Waiting for live packets\u2026', cw/2, ch/2);
      ctx.textAlign = 'left';
      mapDrawScale(ctx, cw, ch); mapDrawLegend(ctx, cw, ch, null, null);
      mapUpdateTimeLbls(); return;
    }
    if (mapVScale <= 0) {
      mapFitView(cw, ch);
      if (mapVScale <= 0) {
        ctx.fillStyle = '#444'; ctx.font = '12px Courier New'; ctx.textAlign = 'center';
        ctx.fillText('No GPS fix \u2014 waiting\u2026', cw/2, ch/2 + 20);
        ctx.textAlign = 'left';
        mapDrawScale(ctx, cw, ch); mapDrawLegend(ctx, cw, ch, null, null);
        mapUpdateTimeLbls(); return;
      }
    }

    // Altitude range for rocket color gradient
    var altVals = pts.filter(function(p){return p.rLat!==null&&p.alt!==null;}).map(function(p){return p.alt;});
    var minAlt = altVals.length ? Math.min.apply(null,altVals) : 0;
    var maxAlt = altVals.length ? Math.max.apply(null,altVals) : 100;

    // Draw 0xBB long-range circles (underneath everything)
    if (lrHistory.length && fullGpsLat !== null) {
      var cellLat = Math.trunc(fullGpsLat);
      var cellLon = Math.trunc(fullGpsLon);
      var now = Date.now();
      var ageMs = now - (mapAgeMax*1000);
      var counts = {};
      for (var i = 0; i < lrHistory.length; i++) {
        var pt = lrHistory[i];
        if (pt.wallTime < ageMs) continue;
        var lf = pt.latFrac, ln = pt.lonFrac;
        if (lf >= 2000 || ln >= 2000) continue;
        var key = lf + ',' + ln;
        counts[key] = (counts[key] || 0) + 1;
      }
      for (var key in counts) {
        var parts = key.split(',');
        var latFrac = parseInt(parts[0]), lonFrac = parseInt(parts[1]);
        var approxLat = cellLat + (latFrac / 2000) * Math.sign(fullGpsLat);
        var approxLon = cellLon + (lonFrac / 2000) * Math.sign(fullGpsLon);
        var cp = mapW2C(approxLat, approxLon, cw, ch);
        var radiusPx = Math.max(2, (25 / 111319) * mapVScale);
        var opacity = Math.min(0.5, 0.05 * counts[key]);
        ctx.beginPath();
        ctx.arc(cp.x, cp.y, radiusPx, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(255,160,0,' + opacity + ')';
        ctx.fill();
      }
    }

    // User-to-rocket lines
    if (mapShowLin) {
      ctx.lineWidth = 0.5;
      pts.forEach(function(p) {
        if (p.type !== 'telem' || p.uLat === null || p.rLat === null) return;
        var u = mapW2C(p.uLat,p.uLon,cw,ch), r = mapW2C(p.rLat,p.rLon,cw,ch);
        ctx.strokeStyle = 'rgba(255,255,255,0.1)';
        ctx.beginPath(); ctx.moveTo(u.x,u.y); ctx.lineTo(r.x,r.y); ctx.stroke();
      });
    }

    // Rocket trajectory: faint path line + altitude-colored triangles
    if (mapShowRkt) {
      ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(255,255,255,0.12)';
      ctx.beginPath(); var firstR = true;
      pts.forEach(function(p) {
        if (p.type !== 'telem' || p.rLat === null) return;
        var s = mapW2C(p.rLat,p.rLon,cw,ch);
        if (firstR) { ctx.moveTo(s.x,s.y); firstR = false; } else ctx.lineTo(s.x,s.y);
      });
      ctx.stroke();
      pts.forEach(function(p) {
        if (p.type !== 'telem' || p.rLat === null) return;
        var s = mapW2C(p.rLat,p.rLon,cw,ch);
        ctx.fillStyle = mapAltColor(p.alt, minAlt, maxAlt);
        ctx.beginPath(); ctx.moveTo(s.x,s.y-5); ctx.lineTo(s.x+4,s.y+4); ctx.lineTo(s.x-4,s.y+4); ctx.closePath(); ctx.fill();
        mapHits.push({x:s.x, y:s.y, pt:p});
      });
    }

    // Missed packet X markers
    if (mapShowMis) {
      ctx.lineWidth = 1.5;
      pts.forEach(function(p) {
        if (!p.isMissed || p.uLat === null) return;
        var s = mapW2C(p.uLat,p.uLon,cw,ch);
        ctx.strokeStyle = '#555';
        ctx.beginPath(); ctx.moveTo(s.x-5,s.y-5); ctx.lineTo(s.x+5,s.y+5);
        ctx.moveTo(s.x+5,s.y-5); ctx.lineTo(s.x-5,s.y+5); ctx.stroke();
        mapHits.push({x:s.x, y:s.y, pt:p});
      });
    }

    // User position dots (SNR/RSSI color)
    if (mapShowUsr) {
      pts.forEach(function(p) {
        if (p.type !== 'telem' || p.uLat === null || p.snr === null) return;
        var s = mapW2C(p.uLat,p.uLon,cw,ch);
        ctx.fillStyle = mapSigColor(p.snr, p.rssi);
        ctx.beginPath(); ctx.arc(s.x, s.y, 4, 0, 6.283); ctx.fill();
        mapHits.push({x:s.x, y:s.y, pt:p});
      });
    }

    // CmdAck squares (rocket's received command link quality)
    if (mapShowAck) {
      pts.forEach(function(p) {
        if (p.type !== 'cmdack' || p.uLat === null) return;
        var s = mapW2C(p.uLat,p.uLon,cw,ch);
        ctx.fillStyle = p.snr !== null ? mapSigColor(p.snr, p.rssi) : '#4af';
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 0.5;
        ctx.fillRect(s.x-4,s.y-4,8,8); ctx.strokeRect(s.x-4,s.y-4,8,8);
        mapHits.push({x:s.x, y:s.y, pt:p});
      });
    }

    mapDrawScale(ctx, cw, ch);
    mapDrawLegend(ctx, cw, ch, minAlt, maxAlt);
    if (altVals.length > 1 && mapShowRkt) mapDrawAltBar(ctx, cw, ch, minAlt, maxAlt);

    // Current user position marker — always on top
    if (mapUserGps) {
      var cp = mapW2C(mapUserGps.lat, mapUserGps.lon, cw, ch);
      if (mapUserGps.acc > 0) {
        var accPx = mapUserGps.acc / 111319 * mapVScale;
        ctx.beginPath(); ctx.arc(cp.x, cp.y, accPx, 0, Math.PI*2);
        ctx.fillStyle = 'rgba(255,255,255,0.05)'; ctx.fill();
        ctx.strokeStyle = 'rgba(255,255,255,0.2)'; ctx.lineWidth = 1; ctx.stroke();
      }
      if (mapDevHeading !== null) {
        var rad = mapDevHeading * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(cp.x + Math.sin(rad)*7, cp.y - Math.cos(rad)*7);
        ctx.lineTo(cp.x + Math.sin(rad)*22, cp.y - Math.cos(rad)*22);
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 2.5; ctx.lineCap = 'round'; ctx.stroke();
        ctx.lineCap = 'butt';
      }
      ctx.beginPath(); ctx.arc(cp.x, cp.y, 6, 0, Math.PI*2);
      ctx.fillStyle = '#fff'; ctx.fill();
      ctx.strokeStyle = '#333'; ctx.lineWidth = 1.5; ctx.stroke();
    }

    mapUpdateTimeLbls();
  }

  function mapDrawScale(ctx, cw, ch) {
    if (mapVScale <= 0) return;
    var mPerDeg = 111319;
    var tgt = cw * 0.15 / mapVScale * mPerDeg;
    var nices = [1,2,5,10,20,50,100,200,500,1000,2000,5000,10000,50000];
    var nice = nices[0];
    for (var i = 0; i < nices.length; i++) if (nices[i] <= tgt) nice = nices[i];
    var px = nice / mPerDeg * mapVScale;
    var bx = 12, by = ch - 14;
    ctx.fillStyle = '#888'; ctx.fillRect(bx, by, px, 2);
    ctx.fillRect(bx, by-4, 1, 6); ctx.fillRect(bx+px, by-4, 1, 6);
    ctx.font = '9px Courier New'; ctx.textAlign = 'left'; ctx.fillStyle = '#888';
    ctx.fillText(nice >= 1000 ? (nice/1000) + 'km' : nice + 'm', bx, by-6);
  }

  function mapDrawLegend(ctx, cw, ch, minAlt, maxAlt) {
    var x = cw - 88, y = 14;
    ctx.font = '9px Courier New'; ctx.textAlign = 'left';
    var items = [
      ['circ', mapSigColor(mapShowSNR ? 5 : -70, -70), mapShowSNR ? 'User SNR' : 'User RSSI'],
      ['tri',  mapAltColor(maxAlt, minAlt || 0, maxAlt || 1), 'Rocket alt'],
      ['sq',   '#4af',  'CmdAck'],
      ['x',    '#555',  'Missed']
    ];
    items.forEach(function(it, i) {
      var lx = x + 6, ly = y + i*14;
      ctx.fillStyle = it[1]; ctx.strokeStyle = it[1]; ctx.lineWidth = 1.5;
      if (it[0] === 'circ') { ctx.beginPath(); ctx.arc(lx,ly,4,0,6.283); ctx.fill(); }
      else if (it[0] === 'tri') { ctx.beginPath(); ctx.moveTo(lx,ly-5); ctx.lineTo(lx+4,ly+3); ctx.lineTo(lx-4,ly+3); ctx.closePath(); ctx.fill(); }
      else if (it[0] === 'sq')  { ctx.strokeStyle='#fff'; ctx.lineWidth=0.5; ctx.fillRect(lx-4,ly-4,8,8); ctx.strokeRect(lx-4,ly-4,8,8); }
      else if (it[0] === 'x')   { ctx.beginPath(); ctx.moveTo(lx-4,ly-4); ctx.lineTo(lx+4,ly+4); ctx.moveTo(lx+4,ly-4); ctx.lineTo(lx-4,ly+4); ctx.stroke(); }
      ctx.fillStyle = '#aaa'; ctx.fillText(it[2], lx+10, ly+3);
    });
    ctx.textAlign = 'left';
    // Signal gradient label
    ctx.fillStyle = '#666'; ctx.font = '9px Courier New';
    ctx.fillText(mapShowSNR ? 'SNR -20\u2192+10dB' : 'RSSI -130\u2192-60dBm', x, y + 4*14 + 4);
  }

  function mapDrawAltBar(ctx, cw, ch, minAlt, maxAlt) {
    var bw = 8, bh = 70, bx = cw - 18, by = ch - bh - 28;
    for (var i = 0; i < bh; i++) {
      ctx.fillStyle = mapAltColor(maxAlt - (maxAlt-minAlt)*i/bh, minAlt, maxAlt);
      ctx.fillRect(bx, by+i, bw, 1);
    }
    ctx.font = '9px Courier New'; ctx.textAlign = 'right'; ctx.fillStyle = '#aaa';
    ctx.fillText(maxAlt.toFixed(0) + 'm', bx - 2, by + 4);
    ctx.fillText(minAlt.toFixed(0) + 'm', bx - 2, by + bh);
    ctx.textAlign = 'left';
  }

  // Tooltip on click
  function mapShowTip(pt) {
    var tip = document.getElementById('map-tip');
    if (!tip) return;
    var d = new Date(pt.wallTime);
    var lines = [
      pad2(d.getHours())+':'+pad2(d.getMinutes())+':'+pad2(d.getSeconds())+' rec#'+(pt.recNum !== null ? pt.recNum : '?'),
      'Phase: '+(pt.phase !== null ? PHASES[pt.phase] : '--')+' ['+pt.type+']',
      'User: '+(pt.uLat !== null ? pt.uLat.toFixed(6)+', '+pt.uLon.toFixed(6)+(pt.uAcc ? ' \u00b1'+pt.uAcc.toFixed(0)+'m' : '') : 'no GPS'),
      'Rocket: '+(pt.rLat !== null ? pt.rLat.toFixed(6)+', '+pt.rLon.toFixed(6) : 'no fix'),
      'FusAlt: '+(pt.alt !== null ? pt.alt+'m' : '--'),
      'SNR: '+(pt.snr !== null ? pt.snr.toFixed(1)+'dB' : '--')+'  RSSI: '+(pt.rssi !== null ? pt.rssi.toFixed(0)+'dBm' : '--')
    ];
    if (pt.type === 'cmdack') lines.push('CmdAck nonce:'+pt.ackNonce+' '+(pt.ackRes===0?'OK':'E'+pt.ackRes));
    if (pt.isMissed) lines.push('[MISSED \u2014 no packet at expected interval]');
    if (pt.pg) {
      var pdef = PD[pt.pg.t];
      var pname = pdef ? pdef.n : '0x'+pt.pg.t.toString(16);
      var pfmt = pdef && pdef.f ? (function(){try{return pdef.f(pt.pg.d);}catch(e){return JSON.stringify(pt.pg.d);}})() : JSON.stringify(pt.pg.d);
      lines.push(pname+': '+pfmt);
    }
    tip.style.display = 'block';
    tip.innerHTML = lines.map(function(l){return '<div>'+l.replace(/&/g,'&amp;').replace(/</g,'&lt;')+'</div>';}).join('');
  }

  // Canvas pointer/touch/wheel handlers
  // Single touch scrolls the page (touch-action:pan-y); 2 touch fingers control map.
  // Mouse drag always pans.
  function mapOnPDown(e) {
    mapPointers[e.pointerId] = {x: e.clientX, y: e.clientY};
    var ids = Object.keys(mapPointers);
    if (e.pointerType !== 'touch' || ids.length >= 2) {
      e.preventDefault();
      this.setPointerCapture(e.pointerId);
    }
    if (ids.length >= 2) {
      // Capture the first touch now that a second has joined
      var first = ids[0] == e.pointerId ? ids[1] : ids[0];
      try { this.setPointerCapture(parseInt(first)); } catch(ex) {}
      var p0 = mapPointers[ids[0]], p1 = mapPointers[ids[1]];
      mapPinchD0 = Math.hypot(p1.x-p0.x, p1.y-p0.y);
      mapPinchCx = (p0.x+p1.x)/2; mapPinchCy = (p0.y+p1.y)/2;
      mapDragMoved = true;
    } else {
      mapDragMoved = false;
    }
  }
  function mapOnPMove(e) {
    if (!mapPointers[e.pointerId]) return;
    var prev = mapPointers[e.pointerId];
    mapPointers[e.pointerId] = {x: e.clientX, y: e.clientY};
    var ids = Object.keys(mapPointers);
    if (ids.length === 1 && e.pointerType !== 'touch') {
      if (!mapDragMoved && Math.hypot(e.clientX-prev.x, e.clientY-prev.y) > 4) mapDragMoved = true;
      mapVOx += e.clientX - prev.x; mapVOy += e.clientY - prev.y;
      mapScheduleDraw();
    } else if (ids.length === 2) {
      var p0 = mapPointers[ids[0]], p1 = mapPointers[ids[1]];
      var nd = Math.hypot(p1.x-p0.x, p1.y-p0.y);
      var nmx = (p0.x+p1.x)/2, nmy = (p0.y+p1.y)/2;
      if (mapPinchD0 > 0 && nd > 0) {
        var rect = document.getElementById('map-canvas').getBoundingClientRect();
        var f = nd / mapPinchD0;
        var ox0 = mapPinchCx - rect.left - rect.width/2;
        var oy0 = mapPinchCy - rect.top  - rect.height/2;
        var ox1 = nmx - rect.left - rect.width/2;
        var oy1 = nmy - rect.top  - rect.height/2;
        mapVOx = ox1 - (ox0 - mapVOx) * f;
        mapVOy = oy1 - (oy0 - mapVOy) * f;
        mapVScale = Math.max(10, mapVScale * f);
      }
      mapPinchD0 = nd; mapPinchCx = nmx; mapPinchCy = nmy;
      mapScheduleDraw();
    }
  }
  function mapOnPUp(e) {
    delete mapPointers[e.pointerId];
    if (Object.keys(mapPointers).length < 2) mapPinchD0 = 0;
  }
  function mapOnWheel(e) {
    e.preventDefault();
    var f = e.deltaY > 0 ? 0.82 : 1.22;
    var rect = document.getElementById('map-canvas').getBoundingClientRect();
    var ox = e.clientX - rect.left - rect.width/2;
    var oy = e.clientY - rect.top  - rect.height/2;
    mapVOx = ox - (ox - mapVOx) * f; mapVOy = oy - (oy - mapVOy) * f;
    mapVScale = Math.max(10, mapVScale * f);
    mapScheduleDraw();
  }
  function mapOnClick(e) {
    if (mapDragMoved) return;
    var canvas = document.getElementById('map-canvas');
    var rect = canvas.getBoundingClientRect();
    var dpr = window.devicePixelRatio || 1;
    var sx = (canvas.width/dpr) / (rect.width || 1);
    var sy = (canvas.height/dpr) / (rect.height || 1);
    var mx = (e.clientX - rect.left) * sx, my = (e.clientY - rect.top) * sy;
    var best = null, bestD2 = 400; // 20px radius
    mapHits.forEach(function(h) {
      var d2 = (h.x-mx)*(h.x-mx) + (h.y-my)*(h.y-my);
      if (d2 < bestD2) { bestD2 = d2; best = h; }
    });
    if (best) mapShowTip(best.pt);
    else { var t = document.getElementById('map-tip'); if (t) t.style.display = 'none'; }
  }

  // Show/hide map panel
  function mapToggle() {
    mapVisible = !mapVisible;
    document.getElementById('map-panel').style.display = mapVisible ? 'block' : 'none';
    document.getElementById('btn-map').className = mapVisible ? 'act' : '';
    if (mapVisible) {
      mapStartGps(); mapStartCompass();
      if (!mapAnimInt) mapAnimInt = setInterval(function() { if (mapUserGps && mapVisible) mapScheduleDraw(); }, 100);
      mapScheduleDraw();
    } else {
      mapStopGps(); mapStopCompass();
      if (mapAnimInt) { clearInterval(mapAnimInt); mapAnimInt = null; }
    }
  }

  // IndexedDB session persistence
  function mapIDBInit() {
    if (!window.indexedDB) return;
    var req = indexedDB.open('moonshot_map', 1);
    req.onupgradeneeded = function(e) {
      var db = e.target.result;
      if (!db.objectStoreNames.contains('sessions'))
        db.createObjectStore('sessions', {keyPath: 'id', autoIncrement: true});
    };
    req.onsuccess = function(e) { mapIDB = e.target.result; mapIDBList(); };
    req.onerror = function() {};
  }
  function mapIDBSave() {
    var st = document.getElementById('map-idb-st');
    if (!mapIDB || !mapPts.length) { if (st) st.textContent = !mapIDB ? 'IDB N/A' : 'No data'; return; }
    var d = new Date(mapPts[0].wallTime);
    var lbl = d.getFullYear()+'-'+pad2(d.getMonth()+1)+'-'+pad2(d.getDate())+' '+pad2(d.getHours())+':'+pad2(d.getMinutes())+' ('+mapPts.length+'pts)';
    var tx = mapIDB.transaction('sessions', 'readwrite');
    tx.objectStore('sessions').add({label: lbl, wallStart: mapPts[0].wallTime, pts: mapPts.slice()});
    tx.oncomplete = function() { if (st) st.textContent = 'Saved'; mapIDBList(); };
    tx.onerror   = function() { if (st) st.textContent = 'Save failed'; };
  }
  function mapIDBList() {
    if (!mapIDB) return;
    var sel = document.getElementById('map-sess-sel');
    if (!sel) return;
    sel.innerHTML = '<option value="">-- saved sessions --</option>';
    var tx = mapIDB.transaction('sessions', 'readonly');
    tx.objectStore('sessions').openCursor(null, 'prev').onsuccess = function(e) {
      var c = e.target.result; if (!c) return;
      var o = document.createElement('option'); o.value = c.value.id; o.textContent = c.value.label;
      sel.appendChild(o); c.continue();
    };
  }
  function mapIDBLoad(id) {
    if (!mapIDB || !id) return;
    mapIDB.transaction('sessions','readonly').objectStore('sessions').get(parseInt(id)).onsuccess = function(e) {
      var rec = e.target.result; if (!rec) return;
      mapPts = rec.pts; mapAckNonces = {}; mapVScale = 0; mapT0 = 0; mapT1 = 1;
      document.getElementById('map-t0').value = 0; document.getElementById('map-t1').value = 100;
      mapUpdateTimeLbls(); mapScheduleDraw();
      var st = document.getElementById('map-idb-st'); if (st) st.textContent = 'Loaded';
    };
  }
  function mapIDBDel(id) {
    if (!mapIDB || !id) return;
    var tx = mapIDB.transaction('sessions', 'readwrite');
    tx.objectStore('sessions').delete(parseInt(id));
    tx.oncomplete = function() { mapIDBList(); var st = document.getElementById('map-idb-st'); if (st) st.textContent = 'Deleted'; };
  }

  window.addEventListener('resize', function() { if (mapVisible) mapScheduleDraw(); });

  // =============================================================
  // INIT
  // =============================================================

  window.addEventListener('load', function() {
    initGrid();
    loadCDN();
    setInterval(function(){ fetchStatus(); }, 30000);

    document.getElementById('btn-cl').addEventListener('click', function() { document.getElementById('log').innerHTML = ''; });
    document.getElementById('btn-cdn').addEventListener('click', loadCDN);
    document.getElementById('btn-fetch').addEventListener('click', loadHistory);
    document.getElementById('btn-stop').addEventListener('click', function() { fetchAbort = true; });
    document.getElementById('btn-prev').addEventListener('click', function() { fetchAutoNav=false; if(viewIdx>0)showView(viewIdx-1);else if(viewIdx===-1&&sessions.length)showView(sessions.length-1); });
    document.getElementById('btn-next').addEventListener('click', function() { fetchAutoNav=false; if(viewIdx>=0&&viewIdx<sessions.length-1)showView(viewIdx+1); });
    document.getElementById('btn-live-base').addEventListener('click', function() { liveSource='base'; showView(-1); });
    document.getElementById('btn-live-rkt').addEventListener('click', function() { liveSource='rkt'; showView(-1); });
    document.getElementById('btn-ble').addEventListener('click', connectBLE);
    document.getElementById('btn-rktble').addEventListener('click', connectRocketBLE);
    document.getElementById('btn-ws-reconnect').addEventListener('click', connectWS);

    // Rocket BLE settings panel toggle
    document.getElementById('btn-rkt-settings').addEventListener('click', function() {
      var p = document.getElementById('rkt-settings-panel');
      p.style.display = p.style.display === 'none' ? 'block' : 'none';
      this.textContent = p.style.display === 'none' ? 'Settings \u25BC' : 'Settings \u25B2';
    });

    // Build page mask checkboxes (pages 1-16)
    (function() {
      var PAGE_NAMES = {1:'GPS Full',2:'Baro',3:'Mag',4:'Accel',5:'Gyro',6:'GPS Ext',7:'Kalman',8:'System',9:'Peaks',10:'Cmd Ack',11:'Flight',12:'Radio',13:'Time',14:'Thrust Curve'};
      // Pages checked by default: 1-13 (matches BLE_DEFAULT_PAGE_MASK bits 1-13).
      // Page 14 (Thrust Curve) is unchecked by default — must be explicitly subscribed.
      var DEFAULT_CHECKED = {1:1,2:1,3:1,4:1,5:1,6:1,7:1,8:1,9:1,10:1,11:1,12:1,13:1};
      var g = document.getElementById('rkt-mask-grid');
      var h = '';
      for (var i = 1; i <= 16; i++) {
        var nm = PAGE_NAMES[i] || ('Pg'+i);
        h += '<label><input type="checkbox" class="rkt-mask-cb" data-page="'+i+'" '+(DEFAULT_CHECKED[i]?'checked':'')+'>'+nm+'</label>';
      }
      g.innerHTML = h;
    })();

    document.getElementById('btn-rkt-rate').addEventListener('click', function() {
      var us = parseInt(document.getElementById('rkt-rate').value) || 0;
      rktSetInterval(us);
    });
    document.getElementById('btn-rkt-phy').addEventListener('click', function() {
      var phy = parseInt(document.getElementById('rkt-phy').value);
      rktSetPhy(phy);
    });
    document.getElementById('btn-rkt-mask-all').addEventListener('click', function() {
      document.querySelectorAll('.rkt-mask-cb').forEach(function(cb){ cb.checked = true; });
    });
    document.getElementById('btn-rkt-mask-none').addEventListener('click', function() {
      document.querySelectorAll('.rkt-mask-cb').forEach(function(cb){ cb.checked = false; });
    });
    document.getElementById('btn-rkt-mask-send').addEventListener('click', function() {
      // Build 64-bit mask as two 32-bit halves (bit N = page N+1, page 1 = bit 1)
      var lo = 0, hi = 0;
      document.querySelectorAll('.rkt-mask-cb').forEach(function(cb) {
        var pg = parseInt(cb.dataset.page);
        if (cb.checked && pg >= 1 && pg <= 32) lo |= (1 << pg);
        else if (cb.checked && pg >= 33 && pg <= 63) hi |= (1 << (pg - 32));
      });
      var buf = new Uint8Array(9);
      buf[0] = RKT_CS_PAGEMASK;
      var dv = new DataView(buf.buffer);
      dv.setUint32(1, lo, true);
      dv.setUint32(5, hi, true);
      if (rktBleConnected && rktConnSetChar_) {
        rktConnSetChar_.writeValueWithoutResponse(buf).catch(function(e) { console.error('mask send failed', e); });
      }
    });
    document.getElementById('btn-ws-reconnect').addEventListener('click', function() {
      // Save base URL to localStorage for next visit
      var url = document.getElementById('base-url').value.trim();
      try { localStorage.setItem('base-url', url); } catch(e) {}
      // Close existing WS and reconnect to new URL
      if (wsObj) { try { wsObj.close(); } catch(e) {} }
      setTimeout(connectWS, 100);
    });

    // Restore base URL: if served from the device, use the device IP; otherwise use saved or default
    (function() {
      var h = window.location.hostname;
      var urlEl = document.getElementById('base-url');
      if (h && h !== 'localhost' && h !== '127.0.0.1' && !h.includes('github.io') && !h.includes('.')) {
        // Probably served from the device itself
        urlEl.value = h;
      } else if (h && /^\d+\.\d+\.\d+\.\d+$/.test(h)) {
        // Direct IP access — use it
        urlEl.value = h;
      } else {
        // Static file or external host — restore saved or use default
        try { var saved = localStorage.getItem('base-url'); if (saved) urlEl.value = saved; } catch(e) {}
      }
    })();
    updateNav();

    document.getElementById('cmd-select').addEventListener('change', onCmdChange);
    document.getElementById('cmd-derive').addEventListener('click', deriveKey);
    document.getElementById('cmd-copykey').addEventListener('click', copyKey);
    document.getElementById('cmd-send').addEventListener('click', sendCommand);
    document.getElementById('btn-detail').addEventListener('click', toggleDetail);
    document.getElementById('btn-voice').addEventListener('click', function() {
      voiceEnabled = !voiceEnabled;
      this.className = voiceEnabled ? 'act' : '';
      document.getElementById('voice-status').textContent = voiceEnabled ? 'on' : '';
      if (!voiceEnabled) { voiceQueue = []; voiceBusy = false; try{speechSynthesis.cancel()}catch(e){} }
    });

    // Default on
    toggleDetail();
    voiceEnabled = true;
    document.getElementById('btn-voice').className = 'act';
    document.getElementById('voice-status').textContent = 'on';
    document.getElementById('voice-mode').addEventListener('change', function() {
      voiceMode = this.value;
    });

    // Nonce: load from server if available (latest ack nonce + 1), fallback to localStorage
    try { var n = localStorage.getItem('cmd-nonce'); if (n) document.getElementById('cmd-nonce').value = n; } catch(e) {}
    document.getElementById('cmd-nonce').addEventListener('change', function() { try { localStorage.setItem('cmd-nonce', this.value); } catch(e) {} });

    // OTA event wiring
    document.getElementById('ota-send').addEventListener('click', sendOtaFirmware);
    document.getElementById('ota-file').addEventListener('change', function() {
      var f = this.files[0];
      if (!f) { otaFileBytes = null; otaFileHmac = null; document.getElementById('ota-file-info').textContent = 'No file selected'; return; }
      document.getElementById('ota-file-info').textContent = f.name + ' (' + f.size + ' bytes)';
      document.getElementById('ota-hmac-status').textContent = 'HMAC: computing...';
      var reader = new FileReader();
      reader.onload = function(ev) {
        otaFileBytes = new Uint8Array(ev.target.result);
        otaUpdateHmac();
      };
      reader.readAsArrayBuffer(f);
    });
    document.getElementById('ota-target').addEventListener('change', function() {
      // Suggest correct target ID when switching devices
      document.getElementById('ota-target-id').value = this.value === 'rkt' ? 146 : 157;
    });

    // Map init
    mapIDBInit();
    document.getElementById('btn-map').addEventListener('click', mapToggle);
    (function() {
      var mc = document.getElementById('map-canvas');
      mc.addEventListener('pointerdown',  mapOnPDown);
      mc.addEventListener('pointermove',  mapOnPMove);
      mc.addEventListener('pointerup',    mapOnPUp);
      mc.addEventListener('pointercancel',mapOnPUp);
      mc.addEventListener('wheel', mapOnWheel, {passive: false});
      mc.addEventListener('click', mapOnClick);
    })();
    document.getElementById('map-snr-btn').addEventListener('click', function() {
      mapShowSNR = !mapShowSNR;
      this.textContent = mapShowSNR ? 'SNR' : 'RSSI';
      this.className = 'mbtn act'; mapScheduleDraw();
    });
    document.getElementById('map-usr-btn').addEventListener('click', function() { mapShowUsr=!mapShowUsr; this.className='mbtn'+(mapShowUsr?' act':''); mapScheduleDraw(); });
    document.getElementById('map-rkt-btn').addEventListener('click', function() { mapShowRkt=!mapShowRkt; this.className='mbtn'+(mapShowRkt?' act':''); mapScheduleDraw(); });
    document.getElementById('map-lin-btn').addEventListener('click', function() { mapShowLin=!mapShowLin; this.className='mbtn'+(mapShowLin?' act':''); mapScheduleDraw(); });
    document.getElementById('map-mis-btn').addEventListener('click', function() { mapShowMis=!mapShowMis; this.className='mbtn'+(mapShowMis?' act':''); mapScheduleDraw(); });
    document.getElementById('map-ack-btn').addEventListener('click', function() { mapShowAck=!mapShowAck; this.className='mbtn'+(mapShowAck?' act':''); mapScheduleDraw(); });
    document.getElementById('map-fit-btn').addEventListener('click', function() { mapVScale=0; mapScheduleDraw(); });
    document.getElementById('map-t0').addEventListener('input', function() {
      mapT0 = parseInt(this.value)/100;
      if (mapT0 > mapT1) { mapT1=mapT0; document.getElementById('map-t1').value=Math.round(mapT1*100); }
      mapUpdateTimeLbls(); mapScheduleDraw();
    });
    document.getElementById('map-t1').addEventListener('input', function() {
      mapT1 = parseInt(this.value)/100;
      if (mapT1 < mapT0) { mapT0=mapT1; document.getElementById('map-t0').value=Math.round(mapT0*100); }
      mapUpdateTimeLbls(); mapScheduleDraw();
    });
    document.getElementById('map-save-btn').addEventListener('click', mapIDBSave);
    document.getElementById('map-load-btn').addEventListener('click', function() {
      var s = document.getElementById('map-sess-sel'); if (s.value) mapIDBLoad(s.value);
    });
    document.getElementById('map-del-btn').addEventListener('click', function() {
      var s = document.getElementById('map-sess-sel');
      if (s.value && confirm('Delete saved session?')) mapIDBDel(s.value);
    });
    document.getElementById('map-live-btn').addEventListener('click', function() {
      mapVScale=0; mapT0=0; mapT1=1;
      document.getElementById('map-t0').value=0; document.getElementById('map-t1').value=100;
      mapUpdateTimeLbls(); mapScheduleDraw();
      var st=document.getElementById('map-idb-st'); if(st) st.textContent='live';
    });
    document.getElementById('map-cmp-btn').addEventListener('click', function() {
      if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        DeviceOrientationEvent.requestPermission().then(function(s) {
          if (s === 'granted') {
            window.addEventListener('deviceorientationabsolute', mapOnDevOrientation, true);
            window.addEventListener('deviceorientation', mapOnDevOrientation, true);
            document.getElementById('map-cmp-btn').style.display = 'none';
          }
        }).catch(function() {});
      }
    });

    mapToggle();
  });

})();