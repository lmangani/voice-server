INTRO
=====

vserver is a simple RTP server. It can terminate, originate and manipulate voice RTP streams.
It is intended as building block for IVR, voice mail, RTP proxy and voice conferencing applications.

API
===

API is JSON-RPC over standard input/output. Server uses notification to inform client of asynchronous
events (like DTMF detection).

Requests
--------

* `create(type[, options, ...])` - creates a resource. First parameter is a resource type. Other parameters a passed to resource
constructor. Returns string containing created resource id.
* `destroy(resourceId)` - destroys a resource. First parameter must be resource id.
* `pull(resourceId, source)` - sets pull source for a resource.
* `push(resourceId, sink)` - sets a push sink for a resource.
* `configure(resourceId[, configuration)` - puts/gets resource configuration. Returns current configuration.

Resources
---------

### RTP

RTP is a RTP session. It can be used as a push source and push sink. Currently its the only configurable resource with following 
configurable properties:

* `local` - local address of a session. It's an object with `address` and `port` properties.
* `remote` - remote address of a session. 
* `rtpmap` - object what maps codec name to RTP payload id. For example `{'PCMA/8000': 8, 'PCMU/8000': 0, 'telephony-events': 121}`

RTP constructor doesn't accept any parameters.

### FileSource

FileSource is a pull source for files. It can be used to play files to RTP sessions. FileSource contructor accepts two parameters: 
filename and codec. File passed to filesource is assumed to be raw file encoded by the codec.

### FileSink

FileSink is push sink for files. It can be used to record RTP sessions. FileSink constructor accepts filename.

### TelephonyEventDetector

TelephonyEventDetector detects DTMF digits carriend by RFC2833 encoded packets. Currently its the only event source. It fires event
`digit(resourceId, digit, timestamp)`.

### JitterBuffer

JitterBuffer is used to connect resources from different clock domains.

Configuring Media Paths
-----------------------

All resources implements at least one of models:

* PushSource - resource has it's own clock and from time to time it produces media packet. Example: RTP sessions.
* PushSink - resource what consumes media packets and doesnt own a clock.
* PullSink - resource what consumes media packets based on its own clock.
* PullSource - resource what produces media packet when asked to.

PushSink can be directly connected to PushSource (`push` method), and PullSink can be directly connected to PullSource (`pull` method).
PushSource can be connected to PullSink via JitterBuffer. Most PushSinks (except JitterBuffer) also implements PullSink model using system
clock.

### Source Descriptors

`pull` method accepts source descriptor as second parameter. Source descriptor is either:

* resource identifier to be used as source. Resource must implement PullSource model.
* array of source descriptors. In this case media from those sources will be mixed.
* transformation descriptor.

There are two possible transformations:
* `{'encode': source, 'codec': codec}` - encodes packets in linear pcm format to be encoded. Non-linear packets are passed through untouched.
* `{'decode': source, 'codec': codec}` - decodes packets from specified codec to linear format. Packets encoded in other formats are passed
through.

### Sink Descriptors

`push` method accepts sink descriptor as second parameter. Sink descriptor is either:

* resource identifier to be used as sink. Resource must implement PushSink model.
* array of sink descriptors. In this case media stream wil be split between those sinks.
* transformation descriptor. Same transfomations as in source descriptor apply.

Examples
--------

### Proxying RTP

Creating a pair of RTP sessions

    --> {'method':'create','params':['rtp']}
    <-- {'result':'rtp0'}
    --> {'method':'create','params':['rtp']}
    <-- {'result':'rtp1'}

and connecting them

    --> {'method':'push','params':['rtp0','rtp1']}
    <-- {'result':null}
    --> {'method':'push','params':['rtp1','rtp0']}
    <-- {'result':null}     

### Recording a RTP session

Creating RTP session

    --> {'method':'create','params':['rtp']}
    <-- {'result':'rtp0'}

FileSink

    --> {'method':'create','params':['filesink', 'record.raw']}
    <-- {'result':'filesink0'}

and JitterBuffer. We can connect FileSink to RTP session directly, but JitterBuffer will compensate for silence periods and lost packets.

    --> {'method':'create','params':['jitterbuffer']}
    <-- {'result':'jitterbuffer0'}

Connecting RTP session to JitterBuffer. Scary object in second parameter inserts a-law and u-law decoders before JitterBuffer.

    --> {'method':'push','params':['rtp0',{'decode':{'decode':'jitterbuffer0', 'codec':'PCMA/8000'}, 'codec':'PCMU/8000'}]}
    <-- {'result': null}

Connecting FileSink to JitterBuffer.

    --> {'method':'pull','params':['filesink0', 'jitterbuffer0']}
    <-- {'result': null}

### Proxing RTP and Recording the Conversation

Creating two RTP sessions

    --> {'method':'create','params':['rtp']}
    <-- {'result':'rtp0'}
    --> {'method':'create','params':['rtp']}
    <-- {'result':'rtp1'}

Creating Filesink,

    --> {'method':'create','params':['filesink', 'record.raw']}
    <-- {'result':'filesink0'}

JitterBuffer.

    --> {'method':'create','params':['jitterbuffer']}
    <-- {'result':'jitterbuffer0'}

And here we are splitting media stream from `rtp0` to `rtp1' and 'jitterbuffer0'

    --> {'method':'push','params':['rtp0',['rtp1', {'decode':{'decode':'jitterbuffer0', 'codec':'PCMA/8000'}, 'codec':'PCMU/8000'}]]}
    <-- {'result':null}

Creating another JitterBuffer

    --> {'method':'create','params':['jitterbuffer']}
    <-- {'result':'jitterbuffer1'}

And connecting second RTP session to its JitterBuffer and first RTP session

    --> {'method':'push','params':['rtp1',['rtp0',{'decode':{'decode':'jitterbuffer1', 'codec':'PCMA/8000'}, 'codec':'PCMU/8000'}]]}
    <-- {'result':null}

Connecting Filesink to mix of JitterBuffers' output

    --> {'method':'pull','params':['filesink0',['jitterbuffer0','jitterbuffer1']]}
    <-- {'result':null}


