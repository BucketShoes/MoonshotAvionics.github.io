Radio timing setup
The radios need a tight timing for a few main reasons:
- short listen window reduce battery usage
- frequency hopping - you need to be in sync to know what channel and modulation the other side will send on


The slot timing system allows for a few main use cases:
1 - short listen windows - reduce battery usage while not listening by knowing exactly when the other side will transmit (if at all). if not syncd, we're just guessing at timing, so we have longer rx windows, and may tx at he wrong time to try to get into sync; but once syncd, theres no point talking any time other than the start of the slot, because the other side wont be listening.

2 - frequency hopping - hopping happens on a per-packet level. sync is needed for this upcoming feature, once basic sync is working reliably. the command channel will not hop. the telemetry channels will hop. regardless, we already have modulation changes. different slots already use different modulation (sf/headers/cr/preamble, etc) - so both sides need to know it

timing:
on boot, all devices start unsync'd.
the rocket will be in long listen mode, to increase the chance of catching an incomming message, to listen almost the entire duration of the WIN_CMD slot (which is about 50% aritime). it doesnt listen other times.
the base station will send a sync command on the command channel. if it doesnt hear telemetry (and hear it at the correct timing), it will resend. the resends are sent not on a multiple of the slot, to try to find when the rocket is atually listening.

on the TxDone/RxDone event of the sync comand, both sides set their anchor time. this time will be used for all future message timing, including deciding what slot type, and when the slot starts. Sending this command will break the existing sync, so do not send when it might still be in sync from previous sync.

the rocket always does everything on simple timing - on the first loop of the slot, do whatever is appropriate for the slot type. However, the base station's responsibility is to track drift, and resync. it starts listening slightly early before the start of the slot, and it sends commands slightly after the start - to allow for slightly incorrect sync.

The rx timeout sent to the sx1262 is how long it will listen in the absence of a packet detection. If a packet is detected, the timeout is ignored, and it will stay active to listen. It will automaically go into standby once the packet has been recieved. This may exceed the slot duration, and this is ok, continue to allow the rx, do not interupt it (i.e. do not call setstandby on the next slot). There is a safety catch if it overflows many slots in a row, but in general, expect a short slot timing and possibly much longer packets. We avoid messages longer than the slot because the following window is skipped because it didnt start on time. For debugging, we have sf5/bw500 for very short airtimes at min power, but the real field use will be closer to 200-800ms airtime. (FCC dwell limits dont apply in AU)

For a successful message, at least a few symbols of the preamble before the header need to be caught. missing the start of the preamble doesn't hurt, but starting to listen after the header will mean unable to recieve it.


WIN_TELEM - the rocket sends a telemetry packet on the next hop channel. the base needs to start listening a little early (to the upcoming channel)
WIN_CMD - the rocket listens on the command channel. the base station usually doesnt send commands very often. If a command is queued, it should send it right at the start of the next WIN_CMD slot - or if the timeout expires, it should send it as soon as the timeout expires (and may send multiple times) - this may be out of sync with the normal timing, and this is deliberate. multiple sends stack end to end are mostly used to recover lost sync.
WIN_LR - this uses a much higher SF and only a specific packet type with much shorter data at a slower rate, no headers, etc for the bare minimum data. the timing and frequency is the same, but different modulation and other radio settings. for implicit headers, the reciever needs to know the params to be able to decode it.

on a regular basis, the base station sends a ping command, which keeps the rocket knowing it's in sync. This doesnt directly effect the sync, but when this command is missing for several minuts, the rocket opens up to longer listening timeouts. this uses more battery, but enables redoing the sync process if needed (with short listen windows, its very hard to send a sync command syccessfully, because the rocket is only listening for a very short time each slot. although the base station is assumed to be with the user (or on a pole, in a known location if serving as a relay), however when the rocket is lost somewhere in a field, we dont have access to reboot it; so it needs to go into a mode automatically where it could sync, without distrupting the existing established sync.

when the base station starts listening early, it must be on the settings of the upcoming slot.


summary:
rocket long rx, base send sync; anchor on rxdone/txdone. rocket acts at the start of each slot, base is early listen/late tx. timeout is only to catch a packet. dont setstandby unless many slots late.

in future, base station will listen message timing of HeaderDone to calibrate for drift. for now, rely on good crystals small drift, and reboot base manually if needed