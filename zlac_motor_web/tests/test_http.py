import json
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app import Backend,make_handler


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.backend=Backend(demo=True)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(
            self.backend,Path(__file__).resolve().parents[1]/'static',set()))
        self.host='127.0.0.1:'+str(self.server.server_port)
        self.server.RequestHandlerClass=make_handler(self.backend,
            Path(__file__).resolve().parents[1]/'static',{self.host})
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        time.sleep(.03)

    def tearDown(self):
        self.server.shutdown();self.thread.join(1);self.server.server_close();self.backend.close()

    def request(self,path,data=None,origin=True):
        headers={'Content-Type':'application/json'}
        if origin:headers['Origin']='http://'+self.host
        request=urllib.request.Request('http://'+self.host+'/api/'+path,json.dumps(data or {}).encode(),headers)
        with urllib.request.urlopen(request,timeout=1) as response:return json.load(response)

    def test_origin_and_host_protection(self):
        with self.assertRaises(urllib.error.HTTPError) as cm:self.request('session',origin=False)
        self.assertEqual(cm.exception.code,403)
        request=urllib.request.Request('http://'+self.host+'/',headers={'Host':'evil.example'})
        with self.assertRaises(urllib.error.HTTPError) as cm:urllib.request.urlopen(request)
        self.assertEqual(cm.exception.code,403)

    def test_demo_http_control_stop_and_dropout(self):
        session=self.request('session');token=session['session']
        a=self.request('arm',dict(session=token,seq=1,ticket=session['ticket']))
        s=self.request('command',dict(session=token,seq=2,ticket=a['ticket'],lease=a['lease'],v=3,w=3))
        self.assertTrue(s['demo']);self.assertTrue(any(w['limited'] for w in s['wheels']))
        self.assertEqual(s['command']['v'],3)
        s=self.request('stop',dict(session=token,seq=3))
        self.assertEqual((s['command']['v'],s['command']['w']),(0,0))
        self.assertFalse(s['controlling'])
        self.request('demo-dropout',dict(session=token))
        time.sleep(.55)
        s=self.request('state',dict(session=token))
        self.assertEqual(s['encoder']['state'],'stale');self.assertFalse(s['ready'])
        self.assertTrue(all(w['measured'] is None and w['error'] is None for w in s['wheels']))
        self.request('demo-dropout',dict(session=token));time.sleep(.03)
        self.assertTrue(self.request('state',dict(session=token))['ready'])


if __name__=='__main__':unittest.main()
