var sip = require('sip');
var sdp = require('sip/sdp');
var vs = require('./vserver');

vs.start({command: '../vserver'}, function() {});

var calls = {};
var cancels = {};

sip.start({}, function(rq) {
  if(rq.headers.to.params.tag) {
    var call = calls[rq.headers.to.params.tag];

    if(call) {
      if(call[rq.method]) {
        call[rq.method](rq);
      }
      else if(rq.method !== 'ACK') {
        sip.send(sip.makeResponse(rq, 405, 'Method Not Allowed'));
      }
    }
    else
      sip.send(sip.makeResponse(rq, 481, 'Call Does Not Exists'));
  }
  else {
    switch(rq.method) {
    case 'INVITE':
      onInvite(rq);
      break;
    default:
      sip.send(sip.makeResponse(rq, 405, 'Method Not Allowed'));
      break;
    }
  }
});

function onInvite(invite) {
  if(!invite.content || (invite.headers['content-type'] && !invite.headers['content-type'].match(/^application\/sdp/))) {
    sip.send(sip.makeResponse(invite, 400, 'Bad Request'));
    return;
  }

  var tag;
  var cancelled = false;
  var rtp;
  var socket;

  cancels[invite.headers.via[0].branch] = function() { cancelled = true; };

  function clear() {
    if(rtp) rtp.close();
    if(socket) socket.close();


    delete cancels[invite.headers.via[0].branch];
    if(tag) delete calls[tag];  
  }

  initRtp(invite.content,
    function(session) {
      rtp = session;
      if(cancelled) {
        clear();
        sip.send(sip.makeResponse(invite, 487, 'Request Terminated'));
        return;
      }

      tag = Math.round(Math.random() * 1e6);

      calls[tag] = {
        BYE: function(rq) {
          clear();
          sip.send(sip.makeResponse(rq, 200, 'OK'));
        }
      };

      var rs = sip.makeResponse(invite, 200, 'OK');
      rs.headers.to.params.tag = tag;
      rs.headers['content-type'] = 'application/sdp';
      rs.headers['content-length'] = rtp.sdp.length;
      rs.content = rtp.sdp;
      sip.send(rs);

      initSocket(rtp, './rtp.socket', 
        function(r) {
          socket = r;
        },
        function() {
          clear();
          sip.send({
            method: 'BYE',
            uri: invite.headers.contact[0].uri,
            headers: {
              to: rs.headers.from,
              from: rs.headers.to,
              'call-id': rs.headers['call-id'],
              cseq: {method: 'BYE', seq: 1}
            }
          });
        }
      );
    },
    function() {
      clear();
      sip.send(sip.makeResponse(invite, 500, 'Server Internal Error'));
    }
  );
}

function initRtp(sdn, success, error) {
  sdn = sdp.parse(sdn);

  vs.request('create', ['rtp'],
    function(r) {
      var rtp = r;

      function clear() { vs.request('destroy', [rtp]); }

      vs.request('configure', [rtp, {rtpmap: {'PCMA/8000': 8}, remote: { address: (sdn.m[0].c || sdn.c).address, port: sdn.m[0].port}}],
        function(r) {
          success({
            id: rtp,
            sdp: sdp.stringify({
              o: {id: Math.round(Math.random() * 1e6), version: Math.round(Math.random() * 1e6), address: r.local.address},
              c: {address: r.local.address},
              m: [{port: r.local.port, fmt:[8]}]
            }),
            close: clear
          });
        },
        function() {
          clear();
          error();
        }
      );
    },
    error
  );
}

function initSocket(rtp, filename, success, error) {
  var socket;
  var jitterbuffer;

  function clear() {
    if(socket) {
      vs.unsubscribe(socket);
      vs.request('destroy', [socket]);
    }
    if(jitterbuffer) vs.request('destroy', [jitterbuffer]);
  }

  vs.request('create', ['jitterbuffer', 'PCMA/8000'],
    function(r) {
      jitterbuffer = r;
      vs.request('create', ['filesocket', filename, 'PCMA/8000'],
        function(r) {
          socket = r;

          vs.subscribe(socket, function(e) {
            if(e === 'connected') {
              vs.request('pull', [socket, jitterbuffer]);
              vs.request('push', [rtp.id, jitterbuffer]);
              vs.request('pull', [rtp.id, socket]);

              success({id: socket, close: clear });
            }
            else if(e === 'error') {
              clear();
              error();
            }
          });
        },
        function() {
          clear();
          error();
        }
      );
    },
    error
  ); 
}

