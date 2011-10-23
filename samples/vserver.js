var util = require('sys');
var spawn = require('child_process').spawn;

var server;
var callback;

exports.start = function start(options, cb) {
  server = spawn(options.command || 'vserver');
  
  server.stdout.setEncoding('utf8');

  server.on('exit', function() { throw new Error('vserver - unexpected exit'); });

  var buffer = ""; 
  server.stdout.on('data', function(data) {
    util.debug(data);

    buffer += data;
    var a = buffer.split('\n');
    util.debug(util.inspect(a));

    buffer = a.pop();

    a.map(JSON.parse).forEach(onMessage);
  });

  callback = cb;
}

function onMessage(m) {
  util.debug(util.inspect(m));

  if(m.id !== undefined && m.id !== null) {
    requests[m.id](m);
  }
  else if(m.params && m.params[0] && subscriptions[m.params[0]]) {
    subscriptions[m.params[0]].apply(null, m.params.slice(1));
  }
  else {
    callback(m.result);
  }
}

var requests = {};
var requestid = 0;
var subscriptions = {};

exports.request = function(method, params, success, error) {
  var rq = {id: requestid++, method: method, params: params};
  requests[rq.id] = function(m) {
    delete requests[m.id];
    ((m.error ? error : success)|| function() {})(m.result);
  };
  
  util.debug(JSON.stringify(rq));
  server.stdin.write(JSON.stringify(rq)+'\r\n');  
  server.stdin.flush();
}

exports.subscribe = function(id, callback) {
  if(callback) {
    subscriptions[id] = callback;
  }
  else {
    delete subscriptions[id];
  }
}

exports.unsubscribe = function(id) {
  exports.subscribe(id, null);
}

