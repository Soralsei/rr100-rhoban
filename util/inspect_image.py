import sys
import json as js
import subprocess as sp

try:
    res = sp.run(['docker', 'inspect', f'{sys.argv[1]}'], capture_output=True)
    meta = js.loads(res.stdout)
    print(meta[0]['Config']['Labels'][sys.argv[2]])
    exit(res.returncode)
except Exception as e:
    print(f"{type(e).__name__}: {e}", file=sys.stderr)
    exit(1)