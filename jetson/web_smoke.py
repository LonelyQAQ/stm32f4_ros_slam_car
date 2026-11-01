import json
import urllib.request

BASE = 'http://127.0.0.1:8080'


def post(obj):
    req = urllib.request.Request(BASE + '/api', data=json.dumps(obj).encode(),
                                 headers={'Content-Type': 'application/json'},
                                 method='POST')
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


with urllib.request.urlopen(BASE + '/', timeout=5) as r:
    body = r.read()
    print('GET /       %d  %d bytes' % (r.status, len(body)))
    print('has canvas:', b'<canvas' in body, '| has WASD:', b'WASD' in body)

with urllib.request.urlopen(BASE + '/webui.js', timeout=5) as r:
    js = r.read()
    print('GET /webui.js %d  %d bytes' % (r.status, len(js)))
    print('has nav_goal:', b'nav_goal' in js, '| has path:', b"'path'" in js)

for cmd in ({'cmd': 'stop'}, {'cmd': 'mode', 'mode': 'manual'},
            {'cmd': 'nav_cancel'}, {'cmd': 'estop'}):
    print('POST', cmd, '->', post(cmd))

with urllib.request.urlopen(BASE + '/stream', timeout=5) as r:
    buf = b''
    while len(buf) < 8000:
        c = r.read(250)
        if not c:
            break
        buf += c
    txt = buf.decode(errors='replace')
    print('SSE head  ', txt[:120].replace('\n', ' | '))
    events = sorted(set(l.split('event: ')[1] for l in txt.splitlines()
                        if l.startswith('event: ')))
    print('events    ', events)
