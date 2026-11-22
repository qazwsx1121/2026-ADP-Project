#!/usr/bin/env python3
"""기본 svg_mppi 설정(yaml)에서 실험용 구성을 만든다. eval_sweep.sh, noise_sweep.sh 가 쓴다.

구성 이름 (모두 트랙 전체에 같은 값, 구간·좌표별 값 없음):
  A  = 부풀림 rigid(공식), guide 끔                       (공식 SVG-MPPI 에 가까운 기준)
  C  = 원형 부풀림 0.15, guide 끔                         (패키지 기본값)
  G  = C + guide 켬, 레이스라인 곡률이 --curv 보다 크면 guide 쉼
  CF = C + 라이다 외톨이 점 제거 필터
  GF = G + 라이다 외톨이 점 제거 필터

사용: python3 gen_cfg.py <base.yaml> <구성> <out.yaml> [--curv 0.65]
"""
import argparse
import re

CFGS = {
    "A": dict(inflation="rigid", guide=False, filt=False),
    "C": dict(inflation="circle", guide=False, filt=False),
    "G": dict(inflation="circle", guide=True, filt=False),
    "CF": dict(inflation="circle", guide=False, filt=True),
    "GF": dict(inflation="circle", guide=True, filt=True),
}


def sub1(pattern, repl, s, what):
    s, n = re.subn(pattern, repl, s)
    if n != 1:
        raise SystemExit(f"gen_cfg: '{what}' not found exactly once in base yaml")
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base")
    ap.add_argument("cfg", choices=sorted(CFGS))
    ap.add_argument("out")
    ap.add_argument("--curv", type=float, default=0.65, help="guide.max_curvature for G/GF [1/m]")
    a = ap.parse_args()
    c = CFGS[a.cfg]
    s = open(a.base).read()
    s = sub1(r"(\n      inflation_mode: )\S+", r"\g<1>" + c["inflation"], s, "local_costmap.inflation_mode")
    s = sub1(r"(\n    guide:\n      enable: )\S+", r"\g<1>" + ("true" if c["guide"] else "false"), s, "guide.enable")
    if re.search(r"\n      max_curvature: ", s):
        s = sub1(r"(\n      max_curvature: )\S+", r"\g<1>" + (f"{a.curv:.3f}" if c["guide"] else "0.0"), s, "guide.max_curvature")
    elif c["guide"]:
        s = sub1(r"(\n      max_range: [^\n]*)", r"\1\n      max_curvature: " + f"{a.curv:.3f}", s, "guide.max_range")
    s = re.sub(r"\n      filter_isolated: [^\n]*\n      filter_radius: [^\n]*", "", s)
    s = sub1(r"(\n      circle_radius: [^\n]*)", r"\1\n      filter_isolated: " + ("true" if c["filt"] else "false")
             + "\n      filter_radius: 0.10", s, "local_costmap.circle_radius")
    open(a.out, "w").write(s)
    print(f"{a.cfg}: {c} curv={a.curv if c['guide'] else 0} -> {a.out}")


if __name__ == "__main__":
    main()
