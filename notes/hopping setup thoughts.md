Hopping setup

the hopping plan combines selection of frequency, sf, bw, preamble, and many other radio settings, in addition to payload restrictions imposed by the window type.

constraints - basically easy for au; but hitting au, eu, fcc in asingle design adds constraints. needs different settings in each country; but same system.

Main plan is usually bw 125 packets - this solves spacing and channel count limits, vs hoping between wide and narrow bands.
 




# packet window types
- bench packet - sf5/bw500 for bench testing only. not a real plan, just hop around a bit and dont annoy anyone too much (lowest airtime/shortest range) all other usage is bw125
- Standard telemetry packet - preamble 6, SF 5-9 depending on settings - set it however far as needed for expected range. payload up to 27 bytes, with a data page
- Long-Range packet - preamble 6, SF 7-11 - no data page, just basic telemetry header (gps pos and flags)
- Find Me packet - preamble 12, SF 10, 4 bytes payload - device id, and maybe some flags or whatever - the payload isnt useful, this exists to be found before timing sync (lack of gps timing implies lack of gps position, so nothing useful to say) - used before reciving time sync; interspersed with standard packets (both at 1/2 interval and +1 sf) - the long preamble makes scanning easier for the recievers to search multiple channels at once



enumerate through 64 channels, with optional skipping. if skipping, e.g. 0,4,8,12,16... then next loop will be 1,5,9,17, etc to cover all channels evenly.

timing window are fixed; but can choose to skip for low power/low interference - all timing windows ar 420ms long.

the idea is, 100/200ms for regular telem, with a once-per minute 399ms packet at a different sf. in the hopping sequence, it cycles the special long-range packet ewvery 63rd packet over a 64 hop seqwuence, about once per minute. gcd(63,64)=1, so this evenly distributes the long packet over all channels.

the device id decides what offset it should have within the 64 cycle. currently its a fixed cycle over a permuttion of frequencies. may consider a PRF indexing into a 64-channel PRP for anti-jamming; but for now, just a counter indexing into a PRP gives us predictable knoweledge that if we hear it on channel X, then Y is next in the sequence, and we've locked on.

decollision - a single fixed permutation does give 1/64 chance that 2 rockets line up on the same hopping and collide every time. Disarmed rockets will randomly check their own tx window for any other RX - if they hear one, they consider changing their offset: if either of the next 2 windows have an incoming, listen up to 4 more - if hearing a total of 3 aligned (3 in a row, or 3 out of 7), we're overlapping on a perfectly aligned sequence, so change to another offset. if hearing any overlap, stay in high listen mode, oytherwise go to rare listen mode.
if hit, check again after low-rate cycles later (e.g. once per minute; but only on a non-LR packet)
i.e. if rnd<0.1 then listen=1 - if hit, then listen+=2; if hit again, listen+=4 if hit yet again then jump. if listen reaches 0, then return to normal. after 5 minutes from boot, of 5 minutes from the last hit, high listen at 10%. after jumping, go into rare listen mode immediately (ddos protection).

 Armed rockets never listen at all (since it would mean not transmitting). Recently booted rockets check much more often in the first 5 minutes. (every listen in their own window means they didnt tx their normal packet, so 10% for first few minutes, then drop to 1% )




 pre-timing sync:
 time sources can either be:
 - boot timer - start from 0 on boot
 - time command - sent by a base station, used mostly if gps is broken. option to cuase this to override gps, but usually only used in the lack of gps. normally not used unless unable to find in search
 - gps time - reliable consistent time, and everyone can sync to it

 after hearing any new time source - send a flag for time in eveyr packet for the next 64 packets then sync

having gps time and gps position are separate cases.


after landing, reduce to rarer tx




 TODO: hopping just slide to time window, or switch in hop window?











window - 417ms, a packet may being at the start of a window, for any duration (up to 400ms dwell limit) a few ms gap for overheads and to make this not align with anyone else using the same idea - e.g. 00:00.0 is probably quite loud globally, for anyone with gps time that wants to send on gps time, we want to be offset from that
multiples are every 417 ms from the start of the gps week rollover (April 7, 2019), or from boot time.

a rocket will tx in one window, and rx in the other (for commands)
a rocket will have its own dedicated command channel (changable by settings) - no hopping. commands are rare enough to not need hopping; and this allows easy recovery.
a base station or relay will listen for the rocket, and stay in sync as its clock drifts. listen from -10ms of expected start - 0.1ms per second since the last one - 10ms for jitter, 0.1ms for drift 1000ppm


# power saving
during flight and on pad, only rx for commands at the window timing; start with jitter window (e.g. 10ms before window). unlike base reciever, rocket has no drift window. real commands come on time. (base may need to spam commands if time is broken, or if rocket has bad time -= base can scan to find rocket timing)
always use CAD to reduce RX power - commands have preamble 8 (CAD 4)

after landing, hold gps running for a while to get best available location, then gps may power cycle to save battery (assume recovery teamin inbound to the general area, but may not know exact position) - clocks will drift slightly. brief vcc wakeup (below acquisition time) should be able to read the 32khz rtc from a vbatt powered gps. longer wakeup can get gps time from satelites. wake gps every 10 minutes until getting a good active tracking gps time (i think thats flag A?)
after landing, reduce tx rate to every REDUCED_TX_RATE window, e.g. every 20 windows - same hopping plan with same hopping timing, i.e. hit channel index 0, then nothing for 19 windows, then hit window 20. A window specifies a time and a frequency - so hitting that window is hitting that window at the right time - to observers, unsent messages are just dropped packets, the timers for which frequency is used when MUST move at the same rate. to keep equal spread, the windows will be at index 0,20,40,60, hit 64 so loop plus 1, the next window will be 80-64+1=17,37,57,14,34 etc. This means each time it hits 64 it will be slightly delayed, and the actually sent packets arent perfectly consistent timing at 1/20th the normal rate.
todo; rate=1 should be 64+1=0, but why =1? should the math all be (63+N+1)%64? rate should should visit all evens then all odds... 62+2=64=0, +1=1....but 63+1=64=0+==1.... we never hit channel 0 after a loop in single skip mode... is it based on the number skipped, not the increment?
1=0,1,2,3,4,5,6...61,62,63,0,1,2,3...
2=0,2,4,6,...,60,62,1,3,5,7...
3=0,3,6,9,...,57,60,63,1,4,7...?
coprimality messes with arbitrary intervals. lets just make it run every integer multiple, from every integer offset; and the looping happens when you hit the offset, skip the potato and move it along by one.
for nonblocking, index++ every time... then tx if its a value we want to send. if its the potato, move the potato back one (mod64)



the "after landing" mode might be reconsidered for any idle - may add a phase for pre-arming, between idle vs ready (do calibration, etc, but not arm pyros, etc; better for if we have long running procresses, like erase a block of flash before moving to armed flag)


scanning for lost timing rocket - the find me packets have a long preamble; so the base station/relays have time to check many channels, e.g. index 0/16/32/48, spending 2 preambol symbol times per frequency, gives enough time to catch anyone transmitting a preamble on any of those - constantly cycling between those 4 channels will mean after only 15 cycles, it should catch one - hearing a preamble, and the 4-byte device id is enough to be able to follow the index to the next index, and just keep listening at the right time (it might not tx in every window, but it does tx, we know where it will do it) 



landed idle powersave:
after definitely landed safe,  (possibly split landed phase into just_landed and landed_snooze) - snooze after sending significant duplicates of a small hdop high-quality gps signal after landing - say several minutes - it might not have been caught, but recovery team will be on their way.

after landed snooze, skip most tx windows, still CAD during command rx windows. sleep cpu, and lower cpu frequency - below 80mhz had some issues in testing, something happened on watchdog - 80 should be ok, but 10 would be lower power, but already much less power than 240mhz.




TODO: potato logic works, but isnt easy to calc; but do we need to?


-------------
rocket must leave space for rx - so only use every 2nd hop
use cases:
startup - could be just after unintended reboot; or could just beturned on
main goal is get found -  findme 400ms + telem 400ms 

preflight - no time, use time since boot. findme+telem




base stations/relays modes:
search mode - look for findme preambles on as many channels at once as possible
found floater rocket without time - base knows rocket's time inferred from getting the packet - look for telem, watch drift
send it the time
also search for other rocket, also check gps windows - depending on power avail on base
recently lost floater - check for gps timed windows
active tracking - of a floater, or of a gps+jitter
can lose a floater by overlap decollision or by power off or by it getting better time or by bad signal (few missed)

pre-gps, we want to find it, but should be short then it gets gps.
still need search/tracking for base reboot or for rocket without time (lost after landed then snooze then big drift)


rocket modes:
startup: send findme + telem - expect base to send time or try to get gps time. every available window will be high airtime.


TODO: how good can we get timing? does data ready work well for different sf/bw/packet sizes? do we need to calc timing? from sync word? windows define the beginning of the preamble

rocket has tx window, and 1 rx window. can skip tx windows - base shouldnt search too hard.
rocket sends flag to say i have better time and ill switch soon





# nope. start over: --------------------------------


# viable packets:
- normal telemetry pre11 header yes crc yes, payload 21 (max24 free on sf10), bw125. sf5=19ms; sf9=197ms sf10=395ms
- radio search pre64 header yes, crc yes, payload 27, bw125. sf8=369ms
- max range - pre12 header no, crc no, payload 4 (id,lat,lon, time or flags or more lat/lon), bw 125. sf11=397ms


should we ditch CRC and add application MAC?
for max range, no CRC, so important to remember history of multiple, in case some were wrong. also need reciever to set len, format, etc for it to work without header






with gps time, make radio search rare (base/relay might have no time)
max range on occasional cycle
normal telem should be usually short, not sf10; max range has option to shorten. but its 500m gps resolution is useless under 1km range

radio search can assume short range finder? it could be on pad, or it could be lost in a swamp after a reboot






could have 64 hannels with 2x64 permutations - theres 2 places it might go next, try both if lost; 1/128 cahnce of perfect overlap. good sync (with offsets) still perfect like prf indexing a prp; but fewer possible places it could hop to - find 2 in a row and youre locked in.


if not time locked, add preamble, on sf8, thats short symbols and lots of time to catch them in a quick scan. reamble 128 might even usuually catch it anywhere in 64 channels in a single packet - todo, need to get practical testing of preamble search; but the 200ms packets just have ptional long preamble if missing gps/time sync



channel exclusion notches - we hop 64 channels evenly; but we can choose to skip some channels by a mask - e.g. to avoid non- hopping traffic. this will just be a non-transmit window, it wont effect the timing of what is when, just we choose never to talk during those. e.g. exclude our own command channel; and add a command to configure which channels to skip (avoid skipping too many; limit max skips. show choices in the order they happen, e.g. tickboxes for 17,42,31,61 ui in the order they aactually happen; and a noe for what freq they are )













do we need gps time, or some persistent sync mechanism, so if we get a sync pre launch, then reboot base station, we can still find the rocket again? or just sync to gps time as soon as we have it? we only need searching if no gps; so sf8 that works within 500m, but we'll never practically catch the sf11 packets. so rare and could be anywhere, it could take hours. maybe the rocket does them more often if the base station hasnt recently told it to stay quiet. we could store rocket hop time vs clock time in persistent on base (or scan logs on base boot), then the base would know where to look, assuming the rocket is still keeping time.
using strict gps time as hop sequence, (rather than just gps time alignment of a random channel) does mean we need every rocket with a serial number, etc to be unique (and overlapping serials would never be fixable, vs reboot is easy fix)

should we stagger when and where to expect preambles? on channel 1, it's on the window. on channel 2 it's 7ms after the window, on channel 3, its 14ms after the window... with no time at all, we cant, but with good time, but maintaining our boot hop timer, disciplined to align to the window, means that anyone that later gets gps can very easily scan for them. it also spreads messages out so 53 rockets in one place dont blast everything at once. but it does shorten our windows, to mean the last channel is transming 53*7 ms late=371ms, is a whole window lost during the hop transition between the highest and lowest channel in the loop. your scheduled time is simultaneous between one window on hop 53 vs the next window on hop 0.



# moderately confident full planfor  hopping:
- 915-928 in 200khz spacing = 64 channels
- use a prime subset: 53 of the 64 channels for hopping rocket tx, with ability to mark channels to avoid, swap for a different one of the remaining channels, e.g. cutouts for other traffic, or the command channel (and other rockets command channels), space for 11 cutouts. aim for most traffic on the standard set from any unsyncronised rocket.
- 420ms windows, always keep tx time under 400ms; and max every 2nd window (the other windows are for commands). more skipped windows for lower power/being nice to neibours.
-after landing, lower to 10 sec intervals.

- rssi scanner mode on base station to find loud stuff - alternate scanning in bw500 and in bw 125 and watch rssi: primitive spectrascope just watching rssi on each

radio search packets and max range packets are sent occasionally. 

- option for an overall fast mode lowers all packets to sf5 for testing. except the max range, always on by default after each boot at sf11 max power every 5 minutes (physical button to turn it to sf5/low power, but reset to full after boot)

-now that ble is working direct to rocket, there's less chance of being unable to fix it if we do lose sync or mismatch our hop planning or change command channels, etc, we can use ble to recover.  add a disarm fallback - if an armed rocket is pointed down while in pad_ready phase. we do support multiple orientations, but we lock it in when arming. (this only applies during pad_ready - any other phase, including landed, stays as-is). we will likely need to disable ble during flight - so this disarm fallback allows manual override (and if it is upside down, we dont want it to launch anyway). need buzzer to indicate status.. turn upside down to disarm and re-enable ble to fix it if lost connection.


## radio/packet types:
- normal telemetry 0xAF: payload max 24, header yes crc yes, bw125. sf5=20ms; sf9=218ms; preamble 11 @ sf10=395ms (selectable depending on needed range. below sf10 uses preamble 12)

- radio search: (also 0xAF) header yes, crc yes, payload 27, bw125. preamble 128 @ sf8=369ms. contains normal telem data, but with extra preamble to find it better in searching (limit sf for airtime and for shorter symbols to find easier - sync only works while close, 500m). reciever scans each channel with setrx with timeout a few symbols long, identifies candidates and rechecks for them each window. possibly with extra data for correlation/hmac/checking its from us not unrelated traffic found in a scan (but mostly that comes from seeing them again in the right place in the next window). extra data for saying prng seed - new packet type, not 0xaf? or 0xaf with flags? or a new data page in 0xaf for the radio sync data and mac - keep crc or disable if we have mac anyway? just allows hardware rejection. add a new data page. the 0xaf content is the same as normal. if normal telem is set to sf8, we also hear these as normal and dont notice the long preamble, but if scanning, we will find it

- max range - preamble 12 header no, crc no, payload 4 (id,lat,lon,flags), bw 125. sf11=397ms - occasional packets on normal channel, but different sf. no verification crc, etc so we'll need to keep track of multiple of these over time in case one is fake/wrong, we dont want the "last recieved location" to actually just be a random point due to junk data, so catch several in one place. either 1 byte lat/lon gives 500m presicion within a 1 degree cell; or more bits for more accurate. but getting within 500m means you can go there, and then hear the normal sf10 signals and use those to actually find it. possibly dither between cells. if at position 151.12345678, take the .12345678*256=31.604736 - so 60% round up and say 32, and 40% round down and say 31. any one packet gets you close, but multiple packets gets even better. still need to exclude junk if we software average this - could just eyeball it on a map with number in each location. probably reserve lat=0 means error (lon=reason), so only 254 wide








 fixed channel for ident and timing, so scan less needed.... but which channel? what if someone is using it? maybe very occasionally
 are we better to send long preamble on various channels, or a regular timing info packet on a fixed channel?
 this gives an easy call and response for bases (or even rockets that hear it) to give the time, non-authoritive
 
 any rocket without time will use the on-time method, whatever that is. no point staying silent (except for longer battery life)
 
 need multiple SF. different range packet types, the normal vs max, can't be on the same SF. max range only fits 4 bytes, and no headers. but normal mode can't go as far. could have a simple command for the baseline for quieter normal telemetry when range not needed, but full big flight will use both at different times.




can't use hop sequence to define SF, gotta be separate, the max range will always be 399ms, so not using channels equally 









# more ideas 2:

if we have 395ms airtime with the radio scan packet, sf8, bw125, preamble approx 140 symbols, giving preamble airtime 280ms then if we scan each channel for 6ms, we have hit 46 channels per preamble, or 66 per whole packet (if we don't catch it until after preamble, we might get something, like hit rsssi, or back invalid packet, etc, which gives us a good candidate, but no data)..
if 6ms scans work, and we listen to the faioures we can catch it on the very first time one is sent while we're doing a continuous scanif they send every 25 sec, then we scan for 25 sec. if we miss it, we just keep waiting. e.g. if anyone else transmits anything on any channel, we'll listen to them instead of scanning; put them as a candidate, but they won't hit the next hop, so the candidate becomes less likely. and we know all rockets will be an exact time later, on a specific channel we can calc from the hopping plan, so  25 sec (or whatever the exact time) after a candidate, we expect a specific time on a specific channel, and look entirely there for that window, or +/- 1 window buffer, instead of scanning; or maybe during that time ever other scan choice is a continuation of a candidate, mixed with scanning unknowns, and spend the other time scanning. it would mean any traffic sets up a 25 seconds echo where we wouldn't hear real rockets because we were looking in the wrong place. and rockets can reduce their timing, so we need to look after 25, 50, 100 sec if they only send every 2,4,8, etc.
we don't get to look during the whole 420ms window, it's only transmitting a findme packet during some of it, and only some of that is a preamble we could catch early and get the data; but 280ms is a good chance. will depend on real world scan results. we don't know when they will tx, so when scanning, were just flipping channels all over the place to find them when they do eventually tx
I was mixed up about the 0/8/16 then 4/12/... we scan x+4 mod 59, so it loops itself with like an interleave. not sure it's necessary to scan +4, its equally likely the be anywhere; so +1 is equivalent. we might alternate between fast scans, like 4ms, and longer scans like 8ms, for more energy to catch them. will need practical testing


depending how many bits we need for radio settings, we might be able to squeeze them into the existing 0xAF header; not sure if we need more than "standard SF", or maybe a mode to say 5/7/9/10 for main and 9/11 for max range packet - 3 bits. but probably a good idea to have things like channel mask/remaps, or some other data anyway, and keep the other one free. if you accidentally catch it by normal telemetry, then you can follow it; although on the main sf, you can't deduce where it is in the 59 cycle (aside from seeing every 59th packet missing); but you do know exactly where to look if you want to find it; but you give up on the possibility of hearing the telem in that cycle is you want to check the sf8 one. the main goal is to catch the hop cycle, then we can get the other info as normally





I think we want to be able to ask questions of the form: for each candidate, what hop index will the next window be on, what type will it be (we might not know yet, if we know the 53 cycle but not the 59 cycle) - i.e. where is it in the 59 type cycle or the 53 hop cycle?, when will it be, how long ago was the last catch, how many hits have we seen/missed (some way to figure out if it was random junk transmission, or actually us, especially if only transmitting occasionally), was it confirmed as us (maybe that becomes a tracked rocket, rather than a candidate) or was it just high rssi/caught unknown mid packet


we need to have a scan mode - look constantly hopping to find unknown candidates, following up in preciously seen candidates on their expected next 59 cycles; but that scan will be constantly interrupted by known rockets, and by potential candidates which we've seen recently.. if we see one, recheck +59 and +59*2,59*4,59*8, but less often as it gets older, then drop it as a candidate.

on holding sync, to make scan work, we don't want to ever shift by much so we can be tracked. once we learn the time, we might hold long-term perfect, based on the random boot time; or a fixed drift. that's probably something to worry about later; but what will be useful up front is id a rocket is confirmed, we have a persistent storage to say when and where it is in the 53&59 cycle relative to GPS time; so after reboot, we don't need to scan, we just know where it is. (in case base station loses power). 
we need to remember the age of the last catch so we can calculate how much drift - if we heard it recently, then only listen a few ms early. if it was a long time ago, listen much earlier.

when not scanning, we want to be able to sleep the radio even on the base station/relays.

for relays, they only need to tx very occasionally. we can have the backhaul non-hopping, and shared by all relays. they use randomised tx time anyway



for the new radio hop data page, it should include at least the hop counter (mod 53) and the packet type counter (mod 59), the SF of each type of packet (5-10 and 5-11). not sure if we want full freedom or just 5/7/9/10 + 9/11 + debug mode bit.. maybe debug mode bit also means bw500, and everything SF5 (and ban outer channels, the extra width puts them outside the band).. it's nice to be able to run debug at short range, but the quietest my radio goes is -9, which on the higher SF is still quiclte some distance


although, I just remembered, the plan is to at max, use every 2nd window. one for tx, one reserved for command listen - that's on another channel, but still means the radio is busy elsewhere. so the 25 seconds cycle is actually minimum 50 seconds. maybe 59 is not the prime we wanted for the packet type cycle length.. perhaps 31? ... although rockets could tx on 7out of every 8.. just makes it harder to hit the command window. we'll also want the base to track their yes/no for each window, if they are choosing to be silent on some of their windows.

often the rocket might be set to only tx every 5 sec (10 windows); and much much less after post landing snooze. we might keep a long prime, but have more special slots; e.g. at 5 mod 59 and 22 mod 59 although that would complicated figuring out if we heard the 5 or the 22 (if we only got rssi/cad, and didn't get data)
we could catch a find me either by getting the packet and data, or just the last half of an invalid packet, unable to decode; either way it's a candidate (or confirmed)
if we put a small 4 byte hmac in the radio scan page, then we know for sure wemhen we find one, but that only works if we caught it during preamble, so candidates are useful. but a good CRC with a bad hmac (or failed to parse) implies it's not from us, so no longer a candidate 


we still want rockets to occasionally listen on their own tx time, if hit too much then completely change to a new hop index


stirring the GPS vs hop list to persistent storage isn't GPS discipline, it's just recovery from reboot. we're still tracking undisciplined transmitters for narrow


make a setting for how long the hop cycle and the packet type cycle is. even when a rocket is only transmitting occasionally, is still scheduled for specific channels at specific times. unless we want to get super aggressive on battery saving, we'll listen to all of them, so it can tx any time. not sure if it's worth having dedicated setup for when it should/shouldn't... maybe at least a dedicated specific slot on the 59 cycle (or 31, or whatever it ends up as) where it will tx nothing, specifically to listen for a command.
the base is easier on power, by the rocket might be more saving. a landed rocket might not even listen during all the quiet windows, but it will always listen during that command slots

if we use 31, that's a bit fast for the max range. everyone is going to hear that for miles around. maybe we just have that more on a every 4th loop of 31 or something.

the rocket and base station and relays will all share the hopping code. maybe switch out the scanning code or unnecessary parts like candidates, or tracking, etc for firmware size, but a common file for timings of a known rockets at least should be common. a rocket can just have one known rocket: itself.

