var util = require('sys');
var spawn = require('child_process').spawn;
var EventEmitter = require('events').EventEmitter;

var server;

exports.start = function start(options) {
  server = spawn(options.command || 'vserver');
  
  server.stdout.setEncoding('utf8');

  var buffer = ""; 
  server.stdout.on('data', function(data) {
    util.debug(data);

    buffer += data;
    var a = buffer.split('\n');
    util.debug(util.inspect(a));

    buffer = a.pop();

    a.map(JSON.parse).forEach(onMessage);
  });
}

function onMessage(m) {
  util.debug(util.inspect(m));

  if(m.id !== undefined) {
    requests[m.id](m);
  }
  else {
    var r = resources[m.params[0]];
    if(r) r.emit(m.method, m.params);
  }
}

var requests = {};
var requestid = 0;

function request(method, params, success, error) {
  var rq = {id: requestid++, method: method, params: params};
  requests[rq.id] = function(m) {
    delete requests[m.id];
    ((m.error ? error : success)|| function() {})(m);
  };
  server.stdin.write(JSON.stringify(rq)+'\r\n');  
  server.stdin.flush();
}

var resources = {};

function Resource(id) {
  this.id = id;
  resources[id] = this;
}

util.inherits(Resource, EventEmitter);

Resource.close = function() { 
  request('close', [this.id]);
  delete resources[this.id];
}

function Rtp(id) { 
  Resource.call(this, id);
};

util.inherits(Rtp, Resource);

Rtp.prototype.configure = function(options, success, error) {
  request("configure", [this.id, options], success, error);
};

var mimeTypes = {
  'PCMA/8000' : 'pcma',
  'PCMA/8000/1' : 'pcma',
  'PCMU/8000' : 'pcmu',
  'PCMU/8000/1': 'pcmu',
  'ILBC/8000': 'ilbc'
};

var reverseMimeTypes = {
  'pcma': 'PCMA/8000',
  'pcmu': 'PCMU/8000',
  'ilbc': 'ILBC/8000'
};

var staticRtpMap = {0: 'pcma', 8: 'pcmu'};

function buildRtpMap(m) {
  var rtpmap = 
    m.a.map(function(x) {
      x.match('^rtpmap:(\d+) (\S+)'); 
    })
    .reduce(function(o, f) {
      if(f && mimeTypes[f[2]])
        o[+f[1]] = mimeTypes[f[2]]; 
      return o;
    }, 
    {}
  );

  return m.fmt.reduce(function(o, f) { 
      if(rtpmap[f] !== undefined)
        o[rtpmap[f]] = f;
      else if(staticRtpMap[f])
        o[staticRtpMap[f]] = f;
      return o;  
    },
    {}
  );
}

Rtp.prototype.setSdp = function(sdp, success, error) {
  var rtpmap = buildRtpMap(sdp.m[0]);
  var that = this;
  
  this.configure({
      rtpmap: rtpmap,
      remote: { address: (sdp.m[0].c || sdp.c).address, port: sdp.m[0].port }
    }, 
    function(rs) {
      if(!that.o) {
        that.o = { id: Math.floor(Math.random() * 1e6), version: Math.floor(Math.random() * 1e6) };
      }

      ++that.o.version;
      that.o.address = rs.result.local.address;

      success({
        o: that.o,
        c: {address: rs.result.local.address},
        m: [
          {
            media: 'audio',
            port: rs.result.local.port,
            fmt: Object.keys(rtpmap).map(function(x) { return rtpmap[x]; }),
            a: Object.keys(rtpmap).map(function(x) { return 'rtpmap:' + rtpmap[x] + ' ' + reverseMimeTypes[x]; })
          }
        ]
      });
    },
    error
  );
};

exports.createRtp = function createRtp(success, error) {
  request(
    'create',
    ['rtp'], 
    function(m) { success(new Rtp(m.result)); },
    error
  );
};

exports.createFileSource = function createFileSource(file, encoding, eof, success, error) {
  request(
    'create',
    ['filesource', file],
    function(m) {
      var r = new Resource(m.result);
      r.on('eof', eof);
      success(r);
    },
    error
  );
};

exports.createFileSink = function createFileSink(file, enconding, success, error) {
  request(
    'create',
    ['filesink', file],
    function(m) { success(new Resource(m.result)); },
    error
  );
};

exports.connect = function connect(source, sink, success, error) {
  request('connect', [source.id, sink.id], success, error);
    
};

exports.disconnect = function disconnect(source, sink, success, error) {
  request('disconnect', [source.id, sink.id], success, error);
};

