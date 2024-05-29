import sys
import json as js
import subprocess as sp

try:
    res = sp.run(['docker', 'inspect', f'{sys.argv[1]}'], capture_output=True)
except Exception as e:
    print(f"{type(e).__name__}: {e}", file=sys.stderr)
    exit(1)

try:
    meta = js.loads(res.stdout)[0]
    print(meta['Config']['Labels'][sys.argv[2]])
except (js.JSONDecodeError, TypeError) as _:
    exit(1)
except KeyError as e:
    print(f'Could not find label "{sys.argv[2]}" in {sys.argv[1]}\'s metadata')
    exit(1)

exit(res.returncode)