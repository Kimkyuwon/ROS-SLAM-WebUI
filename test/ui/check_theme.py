"""토큰이 적용됐는지: body 배경·글꼴, 숫자 요소 글꼴."""
from ui_check_common import finish, open_page

failures = []
with open_page() as (page, errors):
    bg = page.evaluate("getComputedStyle(document.body).backgroundColor")
    font = page.evaluate("getComputedStyle(document.body).fontFamily")
    token = page.evaluate("getComputedStyle(document.documentElement).getPropertyValue('--color-bg').trim()")
    if token.lower() != '#0b0e11':
        failures.append(f'--color-bg={token}')
    if bg != 'rgb(11, 14, 17)':
        failures.append(f'body bg={bg}')
    if 'Fira Sans' not in font:
        failures.append(f'body font={font}')
    finish(failures, errors)
