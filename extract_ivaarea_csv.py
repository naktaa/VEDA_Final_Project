# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import re, csv, sys

# 사용법:
#   python3 extract_ivaarea_csv.py meta_180s.txt ivaarea.csv
# 기본값:
#   in=meta_180s.txt, out=ivaarea.csv

in_path  = sys.argv[1] if len(sys.argv) > 1 else "meta_nomask_180s.txt"
out_path = sys.argv[2] if len(sys.argv) > 2 else "ivaarea2.csv"

# meta 텍스트는 <tt:MetadataStream ...> ... </tt:MetadataStream> 조각이 반복됨
# 그 중 Topic이 tns1:OpenApp/WiseAI/IvaArea 인 블록만 뽑아서 Source/Data의 SimpleItem을 파싱
topic_tag = "tns1:OpenApp/WiseAI/IvaArea"

# 블록 단위 split (XML 선언 포함해서 잘리는 경우가 있어도 최대한 복구)
with open(in_path, "r", errors="ignore") as f:
    s = f.read()

# 각 이벤트 블록 후보: Topic이 포함된 주변을 MetadataStream 단위로 자르기
# (정확한 XML 파싱이 아니라 텍스트 파싱이라, "최대한 안정적으로" 만들기)
chunks = s.split("<?xml version=\"1.0\" encoding=\"UTF-8\"?>")
candidates = []
for ch in chunks:
    if topic_tag in ch:
        candidates.append(ch)

# UtcTime: <tt:Message UtcTime="...">
re_utctime = re.compile(r'<tt:Message[^>]*\sUtcTime="([^"]+)"')
# RuleName: <tt:SimpleItem Name="RuleName" Value="..."/>
re_rule    = re.compile(r'Name="RuleName"\s+Value="([^"]*)"')
# ObjectId / Action / State
re_obj     = re.compile(r'Name="ObjectId"\s+Value="([^"]*)"')
re_act     = re.compile(r'Name="Action"\s+Value="([^"]*)"')
re_state   = re.compile(r'Name="State"\s+Value="([^"]*)"')
# 혹시 intrusion/loitering 같은 문자열이 다른 Key로 오는 경우 대비: SimpleItem 전부도 덤프 가능
re_all_si  = re.compile(r'Name="([^"]+)"\s+Value="([^"]*)"')

rows = []
for ch in candidates:
    utcs = re_utctime.findall(ch)
    # NotificationMessage 안에 Message가 여러 개 들어올 수 있어서 전부 순회
    # 단순히 "가장 먼저 보이는 값"을 하나로 잡는 방식
    utc = utcs[0] if utcs else ""

    rule = (re_rule.findall(ch) or [""])[0]
    obj  = (re_obj.findall(ch)  or [""])[0]
    act  = (re_act.findall(ch)  or [""])[0]
    st   = (re_state.findall(ch)or [""])[0]

    # 필요하면 여기서 전체 SimpleItem을 dict로 만들어 추가 컬럼 뽑기 가능
    # items = dict(re_all_si.findall(ch))

    rows.append([utc, rule, obj, act, st])

# 정렬(시간순) - UtcTime은 ISO라 문자열 정렬로도 어느정도 OK
rows.sort(key=lambda r: r[0])

with open(out_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["UtcTime", "RuleName", "ObjectId", "Action", "State"])
    w.writerows(rows)

print(f"[OK] wrote {len(rows)} rows -> {out_path}")

