"""index.html id 보존 검사: 기준 브랜치 대비 사라진 id와 중복 id를 출력한다.

사용: python3 test/ui/check_ids.py dev [삭제 허용 id ...]
"""
import re
import subprocess
import sys
from collections import Counter

ID_RE = re.compile(r'\bid="([^"]+)"')

base_ref = sys.argv[1] if len(sys.argv) > 1 else 'dev'
allowed = set(sys.argv[2:])
base_html = subprocess.run(['git', 'show', f'{base_ref}:web/index.html'],
                           capture_output=True, text=True, check=True).stdout
with open('web/index.html', encoding='utf-8') as f:
    cur_html = f.read()

removed = sorted(set(ID_RE.findall(base_html)) - set(ID_RE.findall(cur_html)))
dupes = sorted(i for i, n in Counter(ID_RE.findall(cur_html)).items() if n > 1)
bad = [i for i in removed if i not in allowed]
for i in removed:
    print(('ALLOWED ' if i in allowed else 'REMOVED ') + i)
for i in dupes:
    print('DUPLICATE ' + i)
sys.exit(1 if bad or dupes else 0)
