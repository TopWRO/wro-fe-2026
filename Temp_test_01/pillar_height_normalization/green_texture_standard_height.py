#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bottom-anchored standard-height prototype with a photographed standard-pillar template.
Run in the SAME Raspberry Pi Python environment as Recording_N02.py.
S: save; R: sample green pillar; D: debug/mask; C: clear memory; Q/Esc: quit.
"""
import argparse
import base64
from itertools import combinations
from collections import deque
from dataclasses import dataclass, asdict
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time
import cv2
import numpy as np

REFERENCE_PNG_BASE64 = (
    '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'
)

AREA_MODEL = json.loads('{\n  "features": [\n    "log_area_fraction",\n    "rotated_box_width_over_height"\n  ],\n  "classes": [\n    "low",\n    "stand",\n    "tall"\n  ],\n  "mean": [\n    -4.359456660352639,\n    0.586020598065794\n  ],\n  "scale": [\n    1.1981183170764895,\n    0.23803434388625336\n  ],\n  "support_vectors": [\n    [\n      -1.756167978156177,\n      0.7389043350370674\n    ],\n    [\n      -0.9318826857015418,\n      0.35970386897096823\n    ],\n    [\n      -0.7674682293448014,\n      0.4062123537463623\n    ],\n    [\n      -0.13652212742428893,\n      0.8601012467259924\n    ],\n    [\n      1.0041733254866876,\n      2.4086134279481857\n    ],\n    [\n      1.3175261454339218,\n      1.7660591787348252\n    ],\n    [\n      0.3204493686668355,\n      1.1455273204390457\n    ],\n    [\n      0.022298380911437387,\n      0.9559069482768487\n    ],\n    [\n      -1.24852086067829,\n      1.1369085642365973\n    ],\n    [\n      -1.189041743294142,\n      -0.7703815518746261\n    ],\n    [\n      -0.5463587758932321,\n      -0.7461443852477783\n    ],\n    [\n      -0.3638023504673666,\n      -0.7100747728754965\n    ],\n    [\n      -0.15014718027253776,\n      -0.6726895627489886\n    ],\n    [\n      0.2769213745521276,\n      -0.4388319590437182\n    ],\n    [\n      0.5702749762476567,\n      -0.24702191769005805\n    ],\n    [\n      1.0367711453269326,\n      0.37217229081611536\n    ],\n    [\n      1.6320371997921912,\n      1.6837407003346982\n    ],\n    [\n      1.7577585328613634,\n      2.2207682103017783\n    ],\n    [\n      1.467204439858934,\n      1.2095250996065359\n    ],\n    [\n      1.346093534261856,\n      0.9431987339071327\n    ],\n    [\n      1.5954305576613226,\n      1.4585791644692712\n    ],\n    [\n      1.4246261877311612,\n      0.934338357914412\n    ],\n    [\n      0.8880515422352293,\n      0.030000217832075755\n    ],\n    [\n      -0.8311277382426118,\n      -0.5714335318130925\n    ],\n    [\n      -0.8802907915589329,\n      -1.1842748416570477\n    ],\n    [\n      0.48908165641574575,\n      -1.0609932508211404\n    ],\n    [\n      0.7594764408787875,\n      -0.9497398800562732\n    ],\n    [\n      1.1312373058741942,\n      -0.6692010854354171\n    ],\n    [\n      1.5304386940671728,\n      -0.16613777201341856\n    ],\n    [\n      1.6979299943808241,\n      0.11966012241372216\n    ],\n    [\n      2.223990793607819,\n      1.0381020076457592\n    ],\n    [\n      1.5453175640886736,\n      -0.27605784546137657\n    ],\n    [\n      1.1234751856561438,\n      -0.6660353610357755\n    ],\n    [\n      0.7700169233228153,\n      -0.8942447484193528\n    ],\n    [\n      -0.37838741295593975,\n      -0.9946291771666154\n    ],\n    [\n      -0.4687070275531714,\n      -1.0043176248961057\n    ]\n  ],\n  "dual_coef": [\n    [\n      0.7632575894679541,\n      1.1599905342992032,\n      0.0,\n      0.7108486462081981,\n      0.0,\n      10.0,\n      0.0,\n      0.0,\n      0.02627934518390547,\n      -0.1591692701646912,\n      -0.0,\n      -0.0,\n      -0.0,\n      -0.13277326437794082,\n      -0.5201100542260493,\n      -0.20152172562614082,\n      -7.735398916794153,\n      -1.3474601007789402,\n      -0.9829844195770796,\n      -0.4657170501592095,\n      -0.0,\n      -0.0,\n      -0.0,\n      -1.1152413134550574,\n      -0.861424679702496,\n      -0.5063305274123161,\n      -0.17132263981469714,\n      -0.06940082917246143,\n      -0.15607334146830143,\n      -0.1029606757286259,\n      -1.1418896845339839,\n      -0.4399247539996257,\n      -0.0,\n      -0.0,\n      -0.11667265710901638,\n      -0.0\n    ],\n    [\n      0.6380005684286096,\n      0.3392600352776815,\n      0.26412130597906075,\n      0.0,\n      0.4299691474708549,\n      0.9544198774251231,\n      0.3382179289495795,\n      0.38962083645021006,\n      0.21239008896040415,\n      0.26865929729340776,\n      10.0,\n      1.238111819376442,\n      2.1892224631944828,\n      0.0,\n      0.0,\n      0.0,\n      0.0,\n      0.8994332040952729,\n      0.0,\n      0.0,\n      0.2724779229042939,\n      1.9793287200975573,\n      1.7084085626896748,\n      0.0,\n      -0.0,\n      -0.0,\n      -0.0,\n      -0.0,\n      -0.0,\n      -1.7693408662888686,\n      -1.8726176548009246,\n      -0.0,\n      -1.227067948346663,\n      -0.4310137322044354,\n      -3.2556017880102375,\n      -10.0\n    ]\n  ],\n  "intercept": [\n    -0.24208265253605704,\n    -0.024452104180349113,\n    0.0004732686129508967\n  ],\n  "n_support": [\n    9,\n    15,\n    12\n  ],\n  "gamma": 1.0,\n  "training_samples": 102,\n  "training_area_fraction_range": [\n    0.0015592447916666665,\n    0.210400390625\n  ],\n  "training_box_ratio_range": [\n    0.28125,\n    1.15935331506306\n  ],\n  "default_margin": 0.15,\n  "dataset_class_counts": {\n    "low": 31,\n    "stand": 38,\n    "tall": 33\n  }\n}')


STANDARD_RED_TEMPLATE_PNG = (
    'iVBORw0KGgoAAAANSUhEUgAAAFwAAACECAYAAAAdgkwaAAAgAElEQVR4AYTh66/t+WEf5j2f72+ttfc+t7mRI1ISLdG2IpqyZF1s'
    '1ZIap47tNEGcNEXQNg2Kon9ECwToizZBgaJAkAZpgBS9pBejaNG8s8UZcYYayhIjx4piR7IVXUxL4vA2HM713Pde6/f9dK+z58w5'
    'Q1L18+Sll1/1z9Q6ikuhraCOinhaW0+EVkQSS6Lv3HX/D17vWKZlt7NsN7LdZDnZyWYxNossgxESDc2QzcKIjEGiLoUZmniklZIS'
    'l1pFMFNrPNJWS1XrkQop4iguhRahrSSKoK0iLiW0hIgktOqj8tLLr7oSV+qx1gcqnlbBbH1UEd+lRGhZq3/8tvUf/m73d96U7bBc'
    '28luyzIYsZyeWK5fl4REw8xGTk5kt4tlQ6IjstuYm2FuFmMMbY1Zy4xHEmMZxhLrwjpcyZBERVE0cVT1vdTTgmpdqiTaSiKJoK4E'
    'dSWfe/lVcRTUY3WpPlDxWB0ltNW6VEfxlIQWQUS4OBjnq/P/+iv1G7+t3jXOGNutLIsudAztaq6rkTCGiObUehjWA7Jhs8h2Yzk7'
    '4WSjY8gYjGFzsrO9ed2y27HbGifbZEuX6lgYZSyybGRZNMMMcwxzRLF2qiciijoqgmoR4koxEgmtDwVFXnr5Vd+pLtVTKkVcqsfa'
    'SuKR1pU6ihDayIigd87l3Tvu/9pv1X/1O7z3LdmucrYVQ5dpth5LY5mDEcmCjXaj2chuIw2pMYaxHWwXY9lwujWeuWVdFvNkYbcx'
    'rm9YwrLI6ZZlMU5OZbezJlnH0OvXuH5NTk5ksxDUI21VzfpQEd9bgiLxROWll1/1nepSPaWUxKX6XoK22jpK4kpkRETeuevw+3/U'
    'O1/6L3z13a/4oT+8I8vEhrGqjaYoiYHNLBuShaUsWz07ZbuR0oXNsrFZsEzJVrbh9Lr9ZmddKmOQyIhxtmW6NLVUdLs1d6fWj3/M'
    '9oc/le0zz1iunTlqEIq25qyWxlMq4ntKPC0vvfyqD5X6HlrUh0J8t7ZSV+KRlrEMCfnGWx7+2m92/9u/LW+/LYdzQsbQ1npvb//+'
    'HevhHLWojcVRRE011BYngmy2xhLjetlVbFlWc7Nzce1UVUq2G5v9arOfBIm4lLDdGc+9aP+nf9Dy2T9t9+LHkmsn1tYMEmOzNZbF'
    'GBuzTPVYUk9EW1r1gcRjeenlVx21dSW+S4t6JHUU8V1aT2sZiWCk+vo33P/8r3R+8yuu37hGDrqGudcZ+3fv2r/7vodfe8O0t7HY'
    'OjVONmKxHs7NdXWhHpwO24d1uL5x+pCT9SDbyDhhrDrOnO9OXOxYJiOxvajNfjKmbMIYLGGJnt4yP/Nn7X7qR137/u9LTrYO5w/t'
    'z8+N7db2+i3j2nU5PcEgVEilU4Rg1hwxsXZaWxIZoeSll1+hHqnHgiIeaVHUhxIfan1EEY8sYzHPLzicG69/3fmv/nr73rfstovl'
    '+hlZmAfrw73DO7edf/MtF2++qxcPnYxrNtuNPHeNh9O8eOBwvnd/veuBmqbFqWt2romc0m5Z71sPcVvdPYnteZ2YTtSJBcM42ZFo'
    'q8G1F/RHPm3343/GyfPP2t9538V779KFTNtbz9v+0KfsfuCTGbsTbc119fD2Hb04t2SwDJvNRq+dyPVTY7uxb9VRJJHPvfSKuJRQ'
    'qgiKoNSloo7io+opdSVEJOHeQ+PObRf/zX9Tv/f75r23bXdbWQbCiHkxXXz7XRdvvmt9965RthZjRG6eysq8eOhw/9z9/T33rSaG'
    'uOmaaycnci10mPfuWPcH9x3cNw2LrYMTOzsnltOdbE/UqmKu08yZfP8nLZ/+fvt79z34+us2u1h2N4zTYWxvyA/9Kdd+4rNOn3ve'
    '2G4c3r/r27/5j8x79/ThObutay8+b/vpH3D2pz+ZXDuxT0xRl0JeevkVj7XxRD2WulTU99LWIwn1oYwhyNu3ef1rPf+937F5523z'
    'wbvGZpERRE2dXLz9nsOdu+Y7dy2NzXZniJzt2K/mxd7h4bmH5w/du3hotmJxw6lr108tt3baweHcevuhi4sHLuzVatja2tjaGh+7'
    'ZSxbTbVTZ83D0M2JXj8112nevSOzNjeus2Gc3LJ+7EXLD/0pJx//uGTI7Qce/s7ve/j6V2QMm2eu2zxzUz/zKbvPfjonn/qE/W5n'
    '2e3s50orL738iifiqPWBOoqj0nqsCOpSqYqQUJcqYwiWN9518ff/QeebX7GzOtx5xxhhuDTUas7av/Oew/37vH9P7j+0OblmbHZy'
    'umVG9wfrw4f2Dx867Kf14Yo6Pb1pe21j3NhgMFfzwYXD+/ftDw/UUW2cWLJYPn5TsuhgxqVomSYz1v3EamRrc/OUMWTc4NmPces5'
    '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'
    'yZCzEzndym6rLV11rjy44P65rtPYLGw2xvWt5ZnrGnKyY50O79928cZb3Nvbbk6Mm9ct12+y3SraqT03uyex3rtrf/uOxdbu1nPG'
    'jVPWqSNkIDqnee/ceu+B+XBve/O6sdkhLAtdzGVjfPwT9j/34679/E/m5Nln5HO/9IrUI62nVMRR1FHRlrgUjyS0jjbLRkzjrffc'
    'fu1Lffhf/tfy+jfs3/uGxcZQ1Icy2Cw2Lzxn8/FnzAcPXXzzLe5fWDZbEsK4cV1unhpnJwStrqvevq/ne90fjERTcrD5+LOWZ2/J'
    'yQ5xePtd5199g3sHmyzGzRvG7tR47pbstuZcNXsczLma9+7Zv3/bWNndfM64cR1x1CDRWd2v+nBvPb+QDMMi2chuRxaz5NnnzX/+'
    'p5z9/E9l98Jz8tIvvUrrTxJHpR6pK/VEWyOxbBYxbb/xbXf+1i/27u/9nt6/6/DuNyyGKKa6EiHD5rlnbF54xrz7wP7Nt1gPYiMu'
    'ZTGuXbM8f1NOd6SyDNT69m2Hd25zuLBkq6eLbGr74rNy44blxjXJYv/2O86/9iZ3zi0WY7uT7U6unRk3T9ktLOR0o3Pav/ee/dvv'
    'yPlqd+t549qpbBdKWzk9Ybth1nrvocN7d83zC8uytTm7xmbDHGYZz75g/dkfd/Lf+vM5+f4X5aVfehX1oUYdVURCW9SHSj3R1gib'
    'zSC1/dob3vmP/j+98/t/aLepefsbIo4StI6qJJazM+PmDX1wbr17j7mK4ahi7E4sz9w0rp+yMLYLGdZ3b9t/6y3twWKjISdsP/Gc'
    'cf2GzTO3ZGzs337Hxbfetb5z2yJiK8uQ3dY4O5FnzuTazjg9ZQz7t99y/q23uXfPdjkzzs5ks7DbsVlsnrtpXL9hXlw43Lnr8M5t'
    'h3dv25xdt33+WRmLrsxZ45mP6U98xvLnP+3kRz6dvPRLr3paW08ElaA+UARFaFHCZlnItHn9m978d/+Pvf/612yXYcz3aVEUFUdV'
    'QcVQEdREEEQM4/oZCSrbRcZGOx3eeQ97w1Akw+b5m5bnbtncesbYbhxu37H/9nv2b71nmCKGwemJsdsZz1yTW6eWa9dI7N95x8Ub'
    'b1nv3raxsSxncrKVzYaTrc0Lzxg3b+jhYL13bv/mbYf3bhtZ7F54Ts62NOZhyrVn+eyP8sOfcPozn01e+qVXPVGtS5GgVB0lKFof'
    'kaKCZVkYtfmjr/vmv/N/6P23v+LUqeGuK0VRCToRTAwVQR3FYxXElTCGZDBXOrGKoYphbE8tt67ZfuyWsT0x9xfWd+7af/tt7MUQ'
    'l7KR3bA8f8vy3E3j2hmJw7vv2n/7XRfvv2uxsdiIhQyWxfb7XrD9+POq5vnB/o337W/f1fWeYbHZnBnP31CD5ZZ++of0Uy/a/sJP'
    'JC/90queqNalSDzS1lGQ+Ki6VExHy7KQGl/+im/9O/9R77//R07dMDxwpa5MUUwERVQQFFFHdVRH9Z0iGCiCiMU42Rm7nYjOlca8'
    'f1/tRVAssttZbpwZN68ZJzvtNB/sHW7fdXH7HUMMNWxRLLYff97ukx/XVg91eOuOw/t3HC7uGIZhY5yccnLCuGl+8pN86vusP/9j'
    'yUuff5X6LgmtS/VYROJDbWlRCYpMfvfL3vyf/4e9WL/p1A1xTkJL0FVMFEFVEARFVD1WcSU+qigiHosaiIjHYqpVRMRRbGSzld1G'
    'djtC59D93v7+e6KGiiGqYnPjls2Lzxqnp3RxePeO9c49h3vvIeIoZNE+z4ufsPzopzz42T8rL7/yBR+qp1Rdqkv1WARFUIowEvuL'
    'c+v+ofkPfqt3/4O/ZX//G3ZuiD2C6UpFMREUUcOVOqorVcSVqMemiO9UR1XTExFxNBDDYxVVRBwtrkxUEMTiaIwTy62bsttS5n41'
    'zx86PHgf9aFstB/jxgs2f+b73P25H5GXX/mCJOJKldLWUX2g9Z3iShJHF+/f4f133fvil7r+7V+zv/u6rRNxKYNOFBUTRVAMFVfq'
    'qK5UEXUURFAV1HQlgiKivlNREcSVIOqxiMcq6kpEXBlYyFYydO5xwOqxCGPR+TE9ecZ48aY7f/XPyee/8Jor9RH1oap4SmnrsTEG'
    'rcOb73rwO7/X+7/6n5t/77fN+batrTgKpseiakVQBMOVOqo4qqq4EsSViaCIJ+JPVhRBMVRQ3ymIo3qsqqijICKCoY4iIowN8wWr'
    'a7rZevBv/ax8/pdfk/oT1FERV5KYc+qsoySWZdGWN95x++Vf6+1f/Xt89XW8Y2vrKOKoKi4FPXgiiHpa1JV6LK7EY3UUjQ/EE6UV'
    'T8SlhBZxVB8VV6KqCIoFVcVAxSIqVtRRMlTpC+o567LY/09/QV557YtSjxTxWD1WxJW2HqlHEsYSbS1ff8t7/+nf7jt/7x/YbM65'
    '+01jDKlLoUVRUdQTQdRREUd1FPVYXAlCopiiYvqoICqI0oongnqsiMeiHkmpS1UDcSWIiJiCWsWlRFur52zycZtPf9z+b/4lefW1'
    'X0G1RUQR1NOSaKutpyW1jKGm8bU3ffvf/1t9/7d+z+lzW+6+wZxStNSlOoqiqCtBEFeipqKCqMeiqJhiimk4hMNwKY5SgmXWgqGG'
    'GgjiqKiPqsfiiSqi4igGhiAOmKiIJmZX07N88tOu/cJfsPn5H09efe2LPlQfEVQ9ra0IijpalkVN46tveOPf+z/1zpf/wMnJ1pjv'
    'aqc0MovQoqKoJ4IgHiuqKiqO6koNK1ZxMBzwcHCxHQhilIGTle1aG7WooRYVFUVR/2xVQVyJGIKYmB7JoNNqNT1n/uRPeuHf/OuW'
    'H/pE8uprX/ShekoRQdUTdaWUhIwQ/NOv+fr/4j/sw3e+Zjc2xnLblTArM9I6iqI+angiiqKoKIqiy8Y6p7XD3mKvLhL77UKjcxrL'
    'YmCz1lYtc7XptMVGDVMUdaX+/6mqo4GgIiKGYiJkKNYexA84/Et/yff/T/5m3LgmX3jti75LqafUpSIoQetojGirpv7BP/Wt/+X/'
    'vg/e+SMn46ZsznWQDF1XOTAmSVDUIy3iSlyJoqKmiokZqmaGinUOexsXgz3WzYKhdamEMWszajOnzazTWTs1TFHUd6vvVNRRPBFD'
    'DBNFENM1dd3y/Mcc/rW/7MV/41/M9vRUvvDaFx0V8YFSRTxSinhaJZEM6/7CPDy0/4f/qO/9+/+ph+992clyU5ZqQug6jUlWMiKJ'
    'ztUjrSvD0yoqqoqpZihmhllmh3229iPWxMwwM0z1SBizFtOm03bWbrIzDVNM8ScpgqordTRciaOBmKQ0arV63jz9pOVHP+H0b/6c'
    'j/3CT8eyyKuvfVF8D6U+0KiKqIo6GmNBXdy77+L9dz38z3+j5//3v2P/4I9tt9eIRzKGzmmslYkEpZMGRVyJxyoqqqomZmhohjlj'
    'NezHcBhDG20cxnAYroSljE6bOW3WaVe2pkUNRT0tKOJK1ZWoIKgrNUSUgVmrg+ljDp/6s279Sz/r5C/82Vz/05+ybLfy6i9/kXgk'
    'nlLqAw2qrsSVsQzK+t4d9//gD9355S+1X/r71gdfs9meUYQklKyTFkFpEZQM6ilRVFRVFU3MUENFZ6xiJlaxJmaGFVVHEVHLnJZO'
    'W2xMi1oc1T9LFcOVqjqKGCImA5ODvfo+F3/5L/mB/9m/ns0Lz7gI125el1de+6IgrtSV1AeidakI6rFlGerSt971/i/9eu/++m/K'
    'G1+z3v6KzXZLPZJZhBJVl1rUE0E8raKGKqaKoqEZ2mDQqDqIwxItRdH4QI05La1FLRhYHNX3EkcxURXxRBHUEI+MwVytqr7P+X//'
    'r/rk//hfydmNG/brwWazkVdf+6KjoJ5IEZeiRYsQtI6WZahLX3vL2//nX+y93/zHNpu73H1dxvBIMCv1REunjwqCuhI1NKFTVV2p'
    'aILB2FA6p6lmUOpS4pEWNRAREzVEPFbfKUJomYinxVEQZUSLbh2cmdef49/+677/X/sXszs9ta6rtvLKa1+UIijikRRxKTrrkcQj'
    'raOxLIR85U1v/wf/Wd//rX9kd3YwDt/0SF2qzFKPBHWpRT0RV+KxioqaJNq6MlTV0QbDI3EpEpfisaiopkyXpraeFk+rIkJoY6oI'
    'ijiKo4pLY5jzYHWLj/+g3Wd+mH/hJ33sF34mm83GYb93lFd++YuO4rvFpURbH1GPjGUh+MM3vPm/+X/07h/+npMto99GaUmYFZda'
    'lCLoRDwRT0RFRRVVIRR1qcWCgXgs4pFEHE0xUVr1tLgSV+pKEUJLEUfxtCgJicPcqxetP/Fjbv71v+TGT/5z2b34MWPE/nAQ5JVf'
    'fs13ivjegmoJMoaM6pe/6lv/q/+k977+T5xurxu9rZ10eiwuzWKiHouPqqM4qqOoqqMiiKKCIAiJo9QH6igqirpSRDwWj9V3q+9U'
    'xFFcGgtzdbCaPuHir/1lL/4P/0bOXnxBTnaWZTgcVgn5/Bdec5T4iIgn4jsltCWr9Xd/35v/6/+4D779Vbtxk82eFKUTEZcOUzpJ'
    'HaWkCErVlTiqeqyOijiqD2RDoyqCiMeKCmJ6LOoojkrikVYFQ1UcxVFVVBEUEY9k0MVedbxo/Tf/mh/4t/7lbM7OnB8ubLdbs9NR'
    'Xvr8qxIfSuKxiCfiKIkx4mh/sdde2P/273jnf/d/7cN3vm6bMx3TmkmLSCIqs9KikkgZRTzS1mP1tHqiKCIuZWiDiiAirhQVRFFx'
    'JSoqqhnSaaIGhqN4rK7URwVRw3Sm45bND36/9W/+nI//q38l293Ofr/X1lFC/vZLL0uISFyKxAeCaGkric2yGGPQ2t9/oPfveP9X'
    'fq0P/l+fs7/zls24ZnZ16DRN0xQbY4nMSSoJCa04CkHR0kpdiQ+NElOsGK5ERFUEEVeCqKMoKgiGiUlCPVLUQBBU1J8saphqek6f'
    '+6Szn/mM5a/9dG7+1GeMsXHY71U9lr/z8i+hlCqteiyOWo9EjMR2u0Wt7921fvMN77zyK51f+gcu3n/TYqOZVrEm1lFXSoqiGpeq'
    'PpAggkzGrA8FjWXG6DTE8EQEE3UUcTQQFUcVRzEQE0VQVAVxJY6iJLTqqK4EQ8XBxIv6mc949l/+ecuf/3SWT75gs1kc9gfiQ3n5'
    '1S9op7baaqsulSoiIonZamu32Qjy5nvu//1/3Pd/7TeMb71h/843jC50b53sw0WmxKWJaieqLoW2HknEcDTEmEipR9rIdGm12BoZ'
    'okYncWnSVU0RGRvLZJiijiKIgZiYCOpK1PBYFCVD0FbVE4Ns7buqT+hf/Tnf92//d7P5+LPu9+D09NS6rhLaSsgrr/1dbVW1tKVF'
    'VRGPhLa0tptF12n52pvu/X9f6/u/+Y+Nk4fc+7Z5mBz2unKxcLGprqsqSl2a2mhpXYqKiITRSElQqtrBjLaCsQybMuaUZeh6UAdF'
    'ElkWmxnLXEXFURwNERMTcSWOKigiiEsjjjqnqieGOnMwJJ+0/x/88z7xP/ob2V07c+jUlpZ4JMhrv/rrnogniql1qR4JUVFzTvnj'
    'b3rvP/7Pevcf/oGTT270/Nt6ftD9XsuKmamHVddVRkjMTrMxZ8xGRRNKkAwDTaS1jloN7WBORoywlGWWoKuaHhmREZs1llZMUURU'
    'xBC1Iq4EUfFYkJS4FJ0TRVBVB7esnrH5kU/r3/hZn/hXfiHZbhzmas6iqMQjee1Xf50Q8Uj8iYKkoman5Wtvevt/+//s3d/6Xaff'
    'd6L7d3Q/Oex1uhRaWuZUlzrNTBMTLROND6WkdETWab9hv0QbrUtxtJkxZgk6UR1xlLBZWUpUEEcVRH23qGCggihBqYkgjqZp9bzD'
    's59y46//rPEXfsTNn/gzGSc7+3kwZ1GPBfnC3/0S4mkR1CNxpSWRsBnUlD9+w1v/7v+lD/7gy7bPbsVt1qmHg87SECKodpVZTTVU'
    '1aVW1SN1qQhzamuOmKGlZZZZhsUQj2TSKoJgiOEojqKijqI+KqgiFo/FdFT1xELq0IP6pIs/91nP/vf+qrN/7k9l88JNtov9nNqq'
    'ChKP5Au/8iUfiksRR3VUV+JSCJJKqr//T73z7/0nPf/KH9neuM5yoXNyOGgxp6M0dNJJ65G4FFdCPNHSqqmOouKoWFsTbR0F8VjF'
    'ExGPRVBRVByFoFRdWTwWR1VTxFEzKAer+kEP/8pf9My/8S/kxidfNE525v+PMXj/1j0v6MP+en+eZ7YBNIQAACAASURBVO99zpkZ'
    'mOEOAmLEC2qiEcWIRFG8oLmYaJLWNO3qn9D+0qx2JU3T1awmXSvNasxqmrZJxRptbiI6MwwDKooSIyIXkXAdQJwZYO6Xc+acvZ/v'
    '593nci4zgGlfr1Vc3pxpi5IIksg73vkbqnYSIm6oimuSGGHORWwsH/o9j/+df9zT+z5rffOzSbWTuVA6SyuzKC2tvaAhaLUkiK3Y'
    '6bIxFUEQRVVVTTsRxEFEUTfEQVCxUztJHERbB0FIROiCOqiKOrZxTp/1Emc/9Kc850e+K+P8ORcvX7Y6XjMGqnYqiYi8/Z3vogix'
    'FXsRO7VTSkYkbE5Pzc1T5vvf30v/8884+/xnHD3rVrqYnXq2IMzaSYvSybQ1VWjJkFBEUHXVXFTVcBBVFHVQxDPFH60IKnbimrpm'
    'hTqYmGI4qGla3Ka3vNjxn/g6Z9/+Krf+6W+J1cpTp6coCeqa2Erk7e98l2viqniG1laNMVC9+BSPPeLiO9/Vs59/m+WJh63PXTDP'
    'rmirm4Witiq1VTqZRVXsJPHlVGlVVRBK1UFRByFo7YU0vlgVRVDPNFAEw0ExURESWhuL6SU2r/oGN//gd7jpW742N738RTZzunJ2'
    'ZplTUNVZB9VW3v7Od9mJpwlBBdXaqjEGYTz8uNMPfKQX3/Ue80MftDzxgPXRiXn5spq6ma4rWlFaWlVqKyT2QlDXlFZLHdTT1UHt'
    'JbT2UhHP0NqpYKConZoYIhgIgokpSmKv08bG9FVOv/c7Pe+vvCEXXvJ8q5vOGauV2am2WlOl9tpqK2//1XcRYqcIIaitUiSsVkNx'
    '9MCjHv25d/SpD3zEcZ9w5f6PWR+dmJcvmZvJLLEVWqmtSYtoJ7UVYivEVaX22omoOoh6unqmiSLiy6sgvlhNDMkKQxKdUyyYCEHr'
    'FMv6q+XHf8jLfuL7c3zLBU+dncl6yIiqnSIliYS28vZ3vstO1DPFTh0ksVoPVec//4hH/tGb++j7P+T8s1ke+0M9m+blp+xNBGNI'
    'S4vSSWknjetiKwRFS6KdduqaqDqoL6cqaid24praCYI4qJ06iOFgoGJikqGdpnPmzc+3+tpvNL7vO73kB1+T1bkjmzktnWZopyRi'
    'KxFbrZa8/Vd/3U7iy4idqojVemjq+N4HPPB339QnPvARJ+cZx0+aFy/rXFDmtJOxkjjopGVObam9mmKQQWKvtdNOB1FBUVUEdUPV'
    'QRS1E3EQtRMEQVEEccNAUDHJxLD0TD3X2Stf6cLrX+em7/jm3PxVL2Yw0VAl9mIrKFotuftXfk1sxReJZwhjhMH6U/d64G/9k168'
    '5xNWanV0UWx1qq05UckwVitGWBZaWp2T2quKMAaCUldV7URt1VZR19RO7dR0TfxRgiAoouqgCIaonQgjtDY9Uy90+tpXu+3H3pjV'
    'C5/HLeesViur1UrWgxEJSSSxE8RO5O5f+TVfTmzFVhyUkEE+/mlf+Ov/sJcfvM/J+thYX9S5MEurywaLsVobY80YzEknrbbMoqpi'
    'sFpRtG6oBrXXBhNB7dQ1URs1McRORTxTEMQNVTtTEcNORAwSysap+gqbP/d6L/iJH85mrDz42GOMlee+4LnGeiUjxoixGkZCIgiS'
    'yNt++Z3+SIk4SFiWBYt89BMe+Bs/2dNH7nNudYuMMywsG+1CFxkkMcaaoKWlpdWiRVUkIaGuqp26oQ3qhqprYlpUXTNU7MQNww1x'
    'ULVTRAzUwZBEOyy2Tl5i/ic/6Hl//vsyu/LQQw/50O/9PuvhhS95kXM3nXd8fGyshiSEiPV6bZWVvO2X30nrugTVIpEwMmjNK6dy'
    '5ZLT3/33feR/epPN5U87WT1HErLoPGU5FVPWK0nENaVotaVuKGIrKILaaVCUCuqGqmuiqnaKioqIp4u6Jg6qdoqIuC7RTnXi9JZn'
    '8TVf7dyPfK8XvO470vL4o495z++813ve/z6ny8ZsZQxjxMnJsXMnJ86dO3HThQvWq7Xc9Y5f8XQROxVCMMZAefySs/s+78lf/61e'
    'ecuvmJc+5+jogoiazA09E9MYQ8aKBKVlmaq09uqquKEIimhsldIGdUPtVOxU1U7tBMOXqqAI4qB2qiIoiZ2lG3Wbp175x5x/w2vd'
    '9Me/Pre+8uXWR0c2Z6c+e++9fuZn/4V7Pn2PjOHk3HljDAlH5fjoyHq9dnR0JHe9/VdQN4SExE4wxrCzeuhxT/7a+/vEO9+rH/so'
    'V+6zOn+OZWpLS0+NVDJkvZaEOdXWZkOrLUJrL7HXUsRV0dgqtRVt3VAHUVQd1E4QXyx26pp4pjooGailG9OLXPme7/Si/+zP5dxz'
    'bzPOrR0drbV1/+c/703/90/7zB/8oVtve46RYXYaGLOSSoLIW+9+h72gSBBJ7CSM1bBzfN/Dnvypu/v4b36A+QjzARmDZVGlxZTE'
    'KFYrlJZgmcypLUVrbwx7tVXEXqiraq+ualEHQTS26prYakloUcRBUBUE9QypJNpp0w3Hr3T5z3+PF/3EG3Ph5vOSYbUadh5++GF3'
    '3P5WH/v4JxytVpZlMcYw55RQU1udU+68+x3i6UKQ2Ikaq0E4/uwDHv17P9uLH/q49U2LzAcp5qItraqIJCSovYQ5mVNbitbeGCLa'
    '0pJIorZCW2qraif2Oh0EsdPYqtipiNpqUQQhQRhhTloSVYmt6KyZWM6f40VfY77x9V78I9+d4/MnzjYbR+u1nUcfecTvvOe93ve+'
    '9zm9curixYtOTk5oUUklsZM77no78TQhxFYiYbWKhuNPf94jf/uf9tKnPu7kwon0YWZ1LrQ6bZWEEVQS17SlpaXVIkRcU8RBbQWt'
    'Fq2KLy/2gpC4oaW0dVA3DNeNoBiYljlNt+rLX+Hk1d8s3/aqPPtPfr3jcydONxvrMUz15OOPu/8P7/Xe33mvT93zKZeeuuzcyYmd'
    'mkYqQsgdd91tL6itECIkklqNyGB9z30e/m/+t175/Kccr4/k5CKNLmc6y6RIQhAiqtRWCZ2TWcQzxQ1VIWh1FnUQVQdBHERShhuC'
    'WVoaEu1E7WVFay+2ImOo2iwb9VJnr/l2z/mx1+X8V73YeNbNVsdHNstiJJq68tRTLj75hA+8/4N+813v9tSly85fOCcZlmVjjBpB'
    'K7e/9W6UxNNF7IwRIyGL43s+6+H/6n/p6aOfdXx8QY7PWKrd6KzWDSGtvZbSlBGdaO10lkRcE9c0aB1U64ZW7cQNkWAdWmqrzCKu'
    'a90QVYREgtWKudgsi5mvdPpnv9vz/sobctNzbmO9ktWwmYvRsGJzesWy2fjExz/pF978ix575FHPevazjDHMLtIpsVW5/c63ESSe'
    'Lg7GWLEsVj2z+vgn+tBf/weWy19wcvIs1gsTXbTVltpL0NLS0mqqq5UWLbM6K2OQUFslsdfaqSKoCi2t2okbIsEqBLO0tAhCbRXB'
    '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'
    'K0JNndNeoqLznDznBfL8V1je+F2e84ZvzdHRkc3ZhhEVO4mnKSK2gtLaS4ppzmkMxojT0zOnp1f83gc+4B133uXxxx+XO+98m50k'
    'MmIk1pvFuHzJ6Xt+txf/139h89jDji5ckLEhxdRiTmbVVkJLi6IULWMQtPZaMrSTloREldLYa6cWiSa+ROKG2MkYkhC0qM7JLIma'
    'lisbVs+1vOqVjr711W563bfl3CteaIxYlmkqYifxJWIrKC0JQU1UTWMMuliWxac/9Sk/+6af8egjj8idd9xFSGK1Xkni6NKp5d77'
    'Xf53v9Mr//od5pMPW184J2NRRSnmQmlLYq9F3VCEoLbKnGorIbaCUmqnirYamiAkFAkt8aVaZkVYRYTV0C52upmW08n6Jc6+99vd'
    '9qNvyPGLX2jcfM5qvbJZFgdRJLQktorYSVDa2kmonYkagyR27r/3Xv/8n/2Ue++9T+644y47Y8T6aC2Jc49dcvHXfqdX3vd7Nr/1'
    'AfP0UesLxzImaqctc1Laeqa6ob7EnNoyBiP2aqt22potpYkOKmTQkkhL3NAiqM7JrKxXkpCBkugc5pXo6sXOfvz7vPgnfiDj3InL'
    'm8XJuWNzTnuJp4udIigJrdZeQpWSkFEjJfG5++/z5n/zZp/59GfkjjvuQo0xrI/WEi488JiL//xtvfTBj5oPf14vPWh1smbUXqst'
    'nZS2bihFfJF6hpYxNChaWjvtVDHnJNGjYQoJIiUtcVVoUTttUVmt3BASXU5Y3SzPerHTH3u9F/7od2e1Xjs9PbMzRuwk/v8pRUId'
    'jERNq1W0PPDAA9797nf7vQ9+UO64/a12xoj10VpGnb//AY/9/X/Zyx+5z+qmK3rx8zKQEMypnbSUKnXQ2otnCoqgtZdoQktLS6uh'
    'Yi4LoeuVjoGQMCstcVVQEgfVVhIHQXXWXG7Rl7zEeOXXWv/ga/Pc13yDlmWzMUtiLyHqhqgiDuog6iCxFVXKajUQjz76qHs++Qm/'
    '+Zu/KXfc/lY7Y3B0tJbByWc/59G/86Ze/vhnHT97TR/RZSEhYVm0kzkRTezNaW+WIEUIEloHIfYaFC2dlIYm5jJRXa80IcNeia34'
    'UrEVVbFVV9WyLNoX6au/2fnv+lbH3/TVOfmK51mths3ZgpKInYqt2Ktr4otVEXtB7a1WQ3Hp4iUXn3zCr//6r8mdt79VWwlHx2sZ'
    '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'
    'O9Vr/w+8vHKX9OFV5QAAAABJRU5ErkJggg=='
)


@dataclass
class Settings:
    # Same red HSV defaults as Recording_N02.py, including the hue wraparound.
    hue_width: int = 10
    s_min: int = 100
    v_min: int = 70
    min_area: int = 150
    min_size: int = 12
    kernel: int = 3
    weak_s_min: int = 55
    weak_v_min: int = 10
    grow_pixels: int = 4


def seeded_red_mask(rgb, settings, history=None):
    """Strong red seeds select weak-red connected components; fill their holes.

    White/black alone are not positive evidence. Original green is excluded.
    History can seed a dark component, but cannot paint absent/gray background.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    strong = cv2.bitwise_or(
        cv2.inRange(hsv, (0,settings.s_min,settings.v_min), (settings.hue_width,255,255)),
        cv2.inRange(hsv, (180-settings.hue_width,settings.s_min,settings.v_min),(179,255,255)))
    weak = cv2.bitwise_or(
        cv2.inRange(hsv,(0,settings.weak_s_min,settings.weak_v_min),(15,255,255)),
        cv2.inRange(hsv,(165,settings.weak_s_min,settings.weak_v_min),(179,255,255)))
    c = rgb.astype(np.float32)
    weak[(c[:,:,0] <= c[:,:,1]*1.30) | (c[:,:,0] < c[:,:,2]*1.05)] = 0
    weak = cv2.morphologyEx(weak,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    strong = cv2.morphologyEx(strong,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    strong = cv2.morphologyEx(strong,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    selected = np.zeros(weak.shape,np.uint8)
    def grow_local(seed_image,pad,minimum):
        count,labels,stats,_=cv2.connectedComponentsWithStats(seed_image,8)
        for i in range(1,count):
            x,y,w,h,area=stats[i]
            if area<minimum or min(w,h)<settings.min_size:continue
            x0,y0=max(0,x-pad),max(0,y-pad)
            x1,y1=min(weak.shape[1],x+w+pad),min(weak.shape[0],y+h+pad)
            _,local_labels=cv2.connectedComponents(weak[y0:y1,x0:x1],8)
            hit=np.unique(local_labels[labels[y0:y1,x0:x1]==i])
            hit=hit[hit!=0]
            # Bounded growth cannot run along a dark track stripe/wood surface.
            local=np.isin(local_labels,hit).astype(np.uint8)*255
            selected[y0:y1,x0:x1] |= local
    grow_local(strong,settings.grow_pixels,settings.min_area)
    if history is not None:
        # History only retains inside its predicted footprint; it cannot expand
        # by another margin every frame and creep indefinitely into background.
        grow_local(history,0,settings.min_area)
    # Only close small gaps. Fill holes within each selected component, not hulls
    # spanning separate targets and not a whole bounding rectangle.
    selected = cv2.morphologyEx(selected,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours = cv2.findContours(selected,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    filled = np.zeros(selected.shape,np.uint8)
    cv2.drawContours(filled,contours,-1,255,cv2.FILLED)
    return filled, weak, strong

def detect_red(rgb, settings, history=None, prepared=None):
    """Return cleaned mask and up to twenty largest independent red regions.

    Do not swap RGB channels, resize the frame, or merge distant components.
    minAreaRect compensates for modest image-plane roll, NOT 3-D perspective.
    """
    mask, _, strong = prepared if prepared is not None else seeded_red_mask(rgb,settings,history)
    cores = cv2.findContours(strong,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    core_info=[]
    for core in cores:
        moments=cv2.moments(core)
        if moments['m00']>=settings.min_area:
            center=(moments['m10']/moments['m00'],moments['m01']/moments['m00'])
            core_info.append((cv2.contourArea(core),core,center))

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    frame_h, frame_w = mask.shape
    detections = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        foreground = contour
        area = float(cv2.contourArea(contour))
        x, y, w, h = cv2.boundingRect(contour)
        if area < settings.min_area or min(w, h) < settings.min_size:
            continue
        foreground_bbox=[int(x),int(y),int(w),int(h)]
        matches=[entry for entry in core_info if cv2.pointPolygonTest(foreground,entry[2],False)>=0]
        if matches:
            _,contour,_=max(matches,key=lambda entry:entry[0])
            area=float(cv2.contourArea(contour))
            x,y,w,h=cv2.boundingRect(contour)
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        edges = np.roll(box, -1, axis=0) - box
        lengths = np.linalg.norm(edges, axis=1)
        if min(lengths) < 1:
            continue
        # Preserve screen-horizontal width and screen-vertical height. Sorting
        # edges into long/short would lose W/H>1 for the short, wide model.
        horizontal = int(np.argmax(np.abs(edges[:, 0]) / lengths))
        width = float(lengths[horizontal])
        height = float(lengths[(horizontal + 1) % 4])
        polygon = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        vertices = len(polygon)
        outline = "QUAD" if vertices == 4 and cv2.isContourConvex(polygon) else (
            "POLYGON-{}".format(vertices) if 3 <= vertices <= 6 else "IRREGULAR/ROUND")
        fx,fy,fw,fh=foreground_bbox
        clipped = fx<=0 or fy<=0 or fx+fw>=frame_w or fy+fh>=frame_h
        metrics = {
            "id": len(detections) + 1,
            "bbox_xywh": foreground_bbox,
            "shape_bbox_xywh": [int(x),int(y),int(w),int(h)],
            "shape_from_strong_core": bool(matches),
            "axis_width_px": int(w), "axis_height_px": int(h),
            "axis_w_over_h": float(w / h),
            "oriented_width_px": width, "oriented_height_px": height,
            "oriented_w_over_h": width / height,
            "oriented_h_over_w": height / width,
            "contour_area_px": area,
            "rectangle_fill": float(area / (width * height)),
            "polygon_vertices": vertices, "outline": outline,
            "touches_image_border": bool(clipped),
        }
        detections.append({"metrics": metrics, "contour": contour,
                           "foreground_contour": foreground,"box":box,"polygon":polygon})
        if len(detections) == 20:
            break
    return mask, detections


@dataclass
class Rules:
    model_margin: float = 0.15  # SVM decision margin; not a probability.
    edge_pixels: int = 2
    edge_s_min: int = 40
    edge_v_min: int = 15
    switch_frames: int = 5
    tall_scale: float = 2.0/3.0
    low_scale: float = 2.0
    background: str = 'local_white'  # Neighboring white brightness, or literal 'white'.


def ordered_corners(box):
    top, bottom = np.array_split(box[np.argsort(box[:, 1])], 2)
    top, bottom = top[np.argsort(top[:, 0])], bottom[np.argsort(bottom[:, 0])]
    return np.float32([top[0], top[1], bottom[1], bottom[0]])






def average_edge_geometry(detection):
    """Approximate the actual silhouette with a maximum-area inscribed quad.

    Do NOT measure minAreaRect edges: opposite rectangle edges are already equal.
    A multi-face 5/6-corner silhouette has no unique true 4-corner interpretation;
    this is an explicit approximation, with fit quality reported for inspection.
    """
    hull = cv2.convexHull(detection['contour'])
    perimeter = cv2.arcLength(hull, True)
    points = cv2.approxPolyDP(hull, .015 * perimeter, True).reshape(-1,2)
    epsilon = .015
    while len(points) > 12 and epsilon < .10:
        epsilon += .005
        points = cv2.approxPolyDP(hull, epsilon*perimeter, True).reshape(-1,2)
    if len(points) < 4 or len(points) > 12:
        return {'average_quad_valid': False, 'average_w_over_h': None,
                'average_quad_points': [], 'quad_fit_fraction': 0.0}
    quad = max((points[list(ids)].astype(np.float32) for ids in combinations(range(len(points)),4)),
               key=cv2.contourArea)
    # Order top/bottom in the upright box coordinate system, compensating roll.
    matrix = cv2.getPerspectiveTransform(ordered_corners(detection['box']),
                                         np.float32([[0,0],[1,0],[1,1],[0,1]]))
    local = cv2.perspectiveTransform(quad.reshape(-1,1,2), matrix).reshape(-1,2)
    indices = np.argsort(local[:,1])
    upper, lower = indices[:2], indices[2:]
    upper = upper[np.argsort(local[upper,0])]
    lower = lower[np.argsort(local[lower,0])]
    quad = quad[[upper[0],upper[1],lower[1],lower[0]]]
    tl,tr,br,bl = quad
    top, bottom = float(np.linalg.norm(tr-tl)), float(np.linalg.norm(br-bl))
    left, right = float(np.linalg.norm(bl-tl)), float(np.linalg.norm(br-tr))
    width, height = (top+bottom)/2.0, (left+right)/2.0
    fit = float(cv2.contourArea(quad) / max(cv2.contourArea(hull),1.0))
    valid = bool(cv2.isContourConvex(quad) and min(top,bottom,left,right)>3 and fit >= .75)
    return {'average_quad_valid': valid, 'average_quad_points': quad.astype(int).tolist(),
            'edge_top_width_px': top, 'edge_bottom_width_px': bottom,
            'edge_left_height_px': left, 'edge_right_height_px': right,
            'average_width_px': width, 'average_height_px': height,
            'average_w_over_h': width/height if height else None,
            'average_h_over_w': height/width if width else None,
            'quad_fit_fraction': fit}


def infer_area_model(area_fraction, box_ratio):
    """Exact NumPy-only evaluation of the exported 36-support-vector model."""
    model=AREA_MODEL
    z=(np.array([np.log(max(area_fraction,1e-12)),box_ratio])-model['mean'])/model['scale']
    sv=np.asarray(model['support_vectors'])
    kernel=np.exp(-model['gamma']*np.sum((sv-z)**2,axis=1))
    dual=np.asarray(model['dual_coef']);starts=np.r_[0,np.cumsum(model['n_support'])]
    votes=np.zeros(3,int);pair_scores=[];pairs=[];k=0
    for i in range(3):
        for j in range(i+1,3):
            score=float(kernel[starts[i]:starts[i+1]]@dual[j-1,starts[i]:starts[i+1]])
            score+=float(kernel[starts[j]:starts[j+1]]@dual[i,starts[j]:starts[j+1]])
            score+=model['intercept'][k];k+=1
            votes[i if score>0 else j]+=1
            pair_scores.append(score);pairs.append((i,j))
    winner=int(np.argmax(votes))
    margin=min(abs(s) for s,pair in zip(pair_scores,pairs) if winner in pair) if votes[winner]==2 else 0.
    return model['classes'][winner],float(margin)


def classify(detection, rules, frame_shape):
    m = detection['metrics']
    m.update(average_edge_geometry(detection))
    # Identical feature extraction to the training script: filled foreground
    # pixels, normalized by frame size, plus the strong-core rotated-box ratio.
    x,y,w,h=m['bbox_xywh']
    local=np.zeros((h,w),np.uint8)
    contour=detection['foreground_contour']-np.int32([x,y])
    cv2.drawContours(local,[contour],-1,255,cv2.FILLED)
    area=int(cv2.countNonZero(local));fraction=area/(frame_shape[0]*frame_shape[1])
    ratio=float(m['oriented_w_over_h'])
    label,margin=infer_area_model(fraction,ratio)
    m['model_area_pixels']=area;m['model_area_fraction']=fraction
    m['model_label']=label;m['model_margin']=margin
    ar=AREA_MODEL['training_area_fraction_range'];rr=AREA_MODEL['training_box_ratio_range']
    m['model_outside_training_range']=bool(not ar[0]<=fraction<=ar[1] or not rr[0]<=ratio<=rr[1])
    m['classification_ratio']=ratio
    m['decision_reason']='learned_area_and_box_ratio'
    m['partial_view'] = m['touches_image_border']
    if margin < rules.model_margin:
        m['decision_reason']='model_margin_low_hold_previous'
        return 'UNKNOWN-KEEP'
    return {'tall':'TALL-TEXTURE','stand':'NORMAL-KEEP','low':'SHORT-TEXTURE'}[label]


class Stabilizer:
    """Greedy spatial association, motion-aligned masks and confirmed model class changes.

    Tracks never render unmatched historical masks, avoiding green ghost trails.
    It is not an occlusion-aware identity tracker; crossing/merged objects can
    be ambiguous and are part of the real-video validation still needed.
    """
    def __init__(self, rules):
        self.rules = rules
        self.reset()

    def reset(self):
        self.tracks, self.next_id = {}, 1
        self.last_mask = None

    def hint(self, shape):
        hint = np.zeros(shape[:2],np.uint8)
        for t in self.tracks.values():
            if t['missed'] <= 3:
                dx,dy = t['velocity']
                matrix = np.float32([[1,0,dx],[0,1,dy]])
                predicted = cv2.warpAffine(t['active'].astype(np.uint8)*255,
                                          matrix,(shape[1],shape[0]))
                hint = cv2.bitwise_or(hint,predicted)
        return hint

    @staticmethod
    def association(a,b):
        ax,ay,aw,ah = a; bx,by,bw,bh = b
        inter = max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
        iou = inter/max(aw*ah+bw*bh-inter,1)
        distance = np.hypot(ax+aw/2-bx-bw/2,ay+ah/2-by-bh/2)
        scale = max(np.hypot(aw,ah),np.hypot(bw,bh),1)
        area_change = aw*ah/max(bw*bh,1)
        if not .25 <= area_change <= 4 or (iou < .08 and distance > .35*scale):
            return None
        return float(iou - .25*distance/scale)

    def update(self,detections,rgb,settings,weak=None):
        fh,fw = rgb.shape[:2]
        if weak is None:_,weak,_ = seeded_red_mask(rgb,settings)
        pairs=[]
        for i,d in enumerate(detections):
            for tid,t in self.tracks.items():
                predicted = list(t['bbox'])
                predicted[0] += t['velocity'][0]*(t['missed']+1)
                predicted[1] += t['velocity'][1]*(t['missed']+1)
                score = self.association(d['metrics']['bbox_xywh'],predicted)
                if score is not None:pairs.append((score,i,tid))
        assigned,used = {},set()
        for _,i,tid in sorted(pairs,reverse=True):
            if i not in assigned and tid not in used:assigned[i]=tid;used.add(tid)
        for t in self.tracks.values():t['missed']+=1
        self.last_mask = np.zeros((fh,fw),np.uint8)
        for i,d in enumerate(detections):
            m=d['metrics']; raw=m['decision']; ratio=m['classification_ratio']
            current=np.zeros((fh,fw),np.uint8)
            cv2.drawContours(current,[d['foreground_contour']],-1,255,cv2.FILLED)
            bbox=m['bbox_xywh']; center=np.array([bbox[0]+bbox[2]/2,bbox[1]+bbox[3]/2])
            if i not in assigned:
                tid=self.next_id;self.next_id+=1
                t={'bbox':bbox,'center':center,'velocity':np.zeros(2),'missed':0,
                   'prob':current.astype(np.float32)/255,'active':current!=0,'box':ordered_corners(d['box']),
                   'ratios':deque(maxlen=7),'state':raw,'pending':None,'votes':0}
                self.tracks[tid]=t
                previous_active=current!=0
            else:
                tid=assigned[i];t=self.tracks[tid]
                delta=center-t['center']
                # A one-pixel contour protrusion must not move the whole history
                # mask. Small real motion is absorbed by the current-mask update.
                delta[np.abs(delta)<2.0]=0
                matrix=np.float32([[1,0,delta[0]],[0,1,delta[1]]])
                previous=cv2.warpAffine(t['prob'],matrix,(fw,fh))
                previous_active=cv2.warpAffine(t['active'].astype(np.uint8),matrix,(fw,fh),
                                               flags=cv2.INTER_NEAREST)!=0
                t['prob']=.65*previous+.35*(current.astype(np.float32)/255)
                t['velocity']=.5*t['velocity']+.5*delta
                shifted=t['box']+delta
                t['box']=.65*shifted+.35*ordered_corners(d['box'])
                t['center'],t['bbox'],t['missed']=center,bbox,0
            if ratio is not None and raw!='UNKNOWN-KEEP' and not m['partial_view']:
                t['ratios'].append(float(ratio))
            measured=float(np.median(t['ratios'])) if t['ratios'] else ratio
            proposal=t['state']
            if raw!='UNKNOWN-KEEP' and not (m['partial_view'] and t['state']!='UNKNOWN-KEEP'):
                proposal=raw  # Model margin gates evidence; no fixed ratio threshold.
            if proposal!=t['state']:
                t['votes']=t['votes']+1 if t['pending']==proposal else 1
                t['pending']=proposal
                if t['votes']>=self.rules.switch_frames:
                    t['state']=proposal;t['pending']=None;t['votes']=0
            else:t['pending']=None;t['votes']=0
            # Keep moving mask memory only where the CURRENT image supports it.
            # Interior holes are already filled. Strong core comes in immediately.
            near=cv2.dilate(current,np.ones((2*self.rules.edge_pixels+1,2*self.rules.edge_pixels+1),np.uint8))
            allowed=(current!=0)|((near!=0)&(weak!=0))
            stable=((t['prob']>=.55)|(previous_active&(t['prob']>=.25))) & allowed
            t['prob'][~allowed]=0
            t['active']=stable
            d['render_mask']=stable.astype(np.uint8)*255
            d['warp_box']=t['box'].astype(np.float32)
            self.last_mask=cv2.bitwise_or(self.last_mask,d['render_mask'])
            m['raw_decision']=raw;m['decision']=t['state'];m['track_id']=tid
            m['filtered_ratio']=measured;m['pending_votes']=t['votes']
            m['decision_held']=bool(t['state']!=raw)
        self.tracks={tid:t for tid,t in self.tracks.items() if t['missed']<=6}


def prepare_reference(bgra):
    """Keep real photographed shading; extend exterior with nearest green pixel."""
    alpha = bgra[:, :, 3]
    contours = cv2.findContours(alpha, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours or cv2.countNonZero(alpha) < 100:
        raise ValueError('Green reference is too small')
    contour = max(contours, key=cv2.contourArea)
    if min(cv2.minAreaRect(contour)[1]) < 8:
        raise ValueError('Green reference is too narrow')
    rgb = cv2.cvtColor(bgra[:, :, :3], cv2.COLOR_BGR2RGB)
    _, labels = cv2.distanceTransformWithLabels(
        np.where(alpha != 0, 0, 255).astype(np.uint8), cv2.DIST_L2, 5,
        labelType=cv2.DIST_LABEL_PIXEL)
    texture = rgb[alpha != 0][labels - 1]
    corners = ordered_corners(cv2.boxPoints(cv2.minAreaRect(contour)))
    return {'texture': texture, 'corners': corners, 'cutout_bgra': bgra.copy()}


def built_in_reference():
    """The actual third green pillar in the user's approved photograph."""
    data = np.frombuffer(base64.b64decode(REFERENCE_PNG_BASE64), np.uint8)
    return prepare_reference(cv2.imdecode(data, cv2.IMREAD_UNCHANGED))


def sample_reference(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (40, 60, 30), (85, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    fh, fw = mask.shape
    candidates = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if cv2.contourArea(c) >= 150 and min(w, h) >= 12 and x > 0 and y > 0 and x+w < fw and y+h < fh:
            candidates.append(c)
    if not candidates:
        raise ValueError('No complete real green pillar found; existing reference retained')
    c = max(candidates, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    inside = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(inside, [c], -1, 255, cv2.FILLED)
    alpha = cv2.bitwise_and(inside, mask)[y:y+h, x:x+w]
    bgra = np.dstack([cv2.cvtColor(rgb[y:y+h, x:x+w], cv2.COLOR_RGB2BGR), alpha])
    return prepare_reference(bgra)


def standard_reference(green_reference):
    raw=cv2.imdecode(np.frombuffer(base64.b64decode(STANDARD_RED_TEMPLATE_PNG),np.uint8),cv2.IMREAD_UNCHANGED)
    rgb=cv2.cvtColor(raw[:,:,:3],cv2.COLOR_BGR2RGB)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    ref=green_reference['cutout_bgra']
    ghsv=cv2.cvtColor(ref[:,:,:3],cv2.COLOR_BGR2HSV)
    valid=(raw[:,:,3]!=0)&(hsv[:,:,1]>80)
    gvalid=(ref[:,:,3]!=0)&(ghsv[:,:,1]>50)
    source=np.median(hsv[valid],axis=0);target=np.median(ghsv[gvalid],axis=0)
    mapped=hsv.copy()
    mapped[:,:,0]=int(target[0])
    mapped[:,:,1]=np.clip(hsv[:,:,1].astype(np.float32)*target[1]/max(source[1],1),0,255).astype(np.uint8)
    mapped[:,:,2]=np.clip(hsv[:,:,2].astype(np.float32)*target[2]/max(source[2],1),0,255).astype(np.uint8)
    green_rgb=cv2.cvtColor(mapped,cv2.COLOR_HSV2RGB)
    cutout=np.dstack([cv2.cvtColor(green_rgb,cv2.COLOR_RGB2BGR),raw[:,:,3]])
    result=prepare_reference(cutout)
    transform=cv2.getPerspectiveTransform(result['corners'],np.float32([[0,0],[63,0],[63,127],[0,127]]))
    result['canonical_rgb']=cv2.warpPerspective(result['texture'],transform,(64,128),
        flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
    return result

def anchored_height_layer(region,box,reference,scale):
    """Scale the observed silhouette along its body axis about a fixed bottom anchor.

    This is a 2-D approximation; it preserves the anchor, transverse width and
    coherent silhouette, rather than treating side-edge pixels as base contacts.
    """
    fh,fw=region.shape
    ys,xs=np.nonzero(region)
    if not len(xs):return None
    bottom_y=int(ys.max());bottom_xs=xs[ys==bottom_y]
    anchor_x=int(bottom_xs[np.argmin(np.abs(bottom_xs-np.median(bottom_xs)))])
    anchor=np.float32([anchor_x,bottom_y])
    corners=ordered_corners(box)
    u=corners[1]-corners[0];u=u/max(float(np.linalg.norm(u)),1e-6)
    v=np.float32([-u[1],u[0]])
    linear=np.eye(2,dtype=np.float32)+(scale-1)*np.outer(v,v)
    affine=np.column_stack([linear,anchor-linear@anchor]).astype(np.float32)
    new_mask=cv2.warpAffine(region,affine,(fw,fh),flags=cv2.INTER_NEAREST)
    if scale>=1:
        # Extension must cover the complete observed short pillar, including base.
        new_mask|=region
    else:
        # Retain the observed base band, avoiding a shifted foot under perspective.
        direction=v if np.dot(v,corners[3]-corners[0])>=0 else -v
        projected_body=xs*direction[0]+ys*direction[1]
        cutoff=projected_body.max()-0.12*(projected_body.max()-projected_body.min())
        base_pixels=projected_body>=cutoff
        new_mask[ys[base_pixels],xs[base_pixels]]=255
    new_mask[bottom_y+1:]=0
    new_mask[bottom_y,anchor_x]=255
    points=cv2.findNonZero(new_mask)
    target=ordered_corners(cv2.boxPoints(cv2.minAreaRect(points)))
    source=np.float32([[0,0],[63,0],[63,127],[0,127]])
    transform=cv2.getPerspectiveTransform(source,target)
    layer=cv2.warpPerspective(reference['canonical_rgb'],transform,(fw,fh),
        flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
    erase=cv2.bitwise_and(region,cv2.bitwise_not(new_mask))
    old_bounds=list(cv2.boundingRect(region));new_bounds=list(cv2.boundingRect(new_mask))
    projected=np.column_stack([xs,ys,np.ones(len(xs))])@affine.T
    info={'height_normalized':True,'requested_height_scale':float(scale),
          'anchor_bottom_xy':[anchor_x,bottom_y],
          'anchor_preserved':bool(new_mask[bottom_y,anchor_x]!=0),
          'old_bbox_xywh':old_bounds,'normalized_bbox_xywh':new_bounds,
          'normalized_top_clipped':bool(np.any(projected[:,1]<0)|np.any(projected[:,0]<0)|np.any(projected[:,0]>=fw)),
          'removed_pixels':int(cv2.countNonZero(erase))}
    return new_mask,layer,erase,info

def white_background(rgb,region,mode):
    if mode=='white':return np.uint8([255,255,255])
    ring=cv2.dilate(region,np.ones((21,21),np.uint8))
    ring[region!=0]=0
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    samples=rgb[(ring!=0)&(hsv[:,:,1]<65)&(hsv[:,:,2]>80)]
    return np.median(samples,axis=0).astype(np.uint8) if len(samples)>=20 else np.uint8([235,235,235])

def process_frame(rgb,reference,settings=None,rules=None,tracker=None):
    begin=time.perf_counter()
    settings,rules=settings or Settings(),rules or Rules()
    history=tracker.hint(rgb.shape) if tracker is not None else None
    prepared=seeded_red_mask(rgb,settings,history)
    _,detections=detect_red(rgb,settings,history,prepared)
    detections.sort(key=lambda d:d['metrics']['bbox_xywh'][0])
    for index,d in enumerate(detections,1):
        m=d['metrics'];m['id']=index;m['decision']=classify(d,rules,rgb.shape)
        m['changed_pixels']=0;m['height_normalized']=False
    if tracker is not None:tracker.update(detections,rgb,settings,prepared[1])
    classified=time.perf_counter()
    protected=np.zeros(rgb.shape[:2],np.uint8)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    protected=cv2.inRange(hsv,(40,50,20),(85,255,255))
    for d in detections:
        if d['metrics']['decision'].endswith('KEEP'):
            if 'render_mask' in d:protected|=d['render_mask']
            else:cv2.drawContours(protected,[d['foreground_contour']],-1,255,-1)
    layers=[]
    for d in detections:
        m=d['metrics']
        if not m['decision'].endswith('TEXTURE'):continue
        region=d.get('render_mask')
        if region is None:
            region=np.zeros(rgb.shape[:2],np.uint8)
            cv2.drawContours(region,[d['foreground_contour']],-1,255,-1)
        scale=rules.tall_scale if m['decision']=='TALL-TEXTURE' else rules.low_scale
        result=anchored_height_layer(region,d.get('warp_box',d['box']),reference,scale)
        if result is None:continue
        new_mask,layer,erase,info=result
        new_mask[protected!=0]=0;erase[protected!=0]=0
        m.update(info)
        m['background_fill_rgb']=white_background(rgb,region,rules.background).tolist()
        layers.append((d,new_mask,layer,erase))
    out=rgb.copy();changed=np.zeros(rgb.shape[:2],np.uint8)
    removed=np.zeros(rgb.shape[:2],np.uint8);normalized=np.zeros(rgb.shape[:2],np.uint8)
    # Erase all surplus old pillar pixels before compositing any new pillar.
    for d,new_mask,layer,erase in layers:
        out[erase!=0]=d['metrics']['background_fill_rgb'];removed|=erase
    for d,new_mask,layer,erase in sorted(layers,key=lambda item:item[0]['metrics']['anchor_bottom_xy'][1]):
        out[new_mask!=0]=layer[new_mask!=0]
        normalized|=new_mask;changed|=(new_mask|erase)
        d['metrics']['changed_pixels']=int(cv2.countNonZero(new_mask|erase))
    done=time.perf_counter()
    timing={'analysis_ms':(classified-begin)*1000,'height_render_ms':(done-classified)*1000,
            'frame_processing_ms':(done-begin)*1000}
    for d in detections:d['metrics']['frame_timing_ms']=timing
    process_frame.last_timing=timing
    process_frame.last_removed_mask=removed;process_frame.last_normalized_mask=normalized
    return out,changed,detections


def debug_view(rgb, detections):
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    fh,fw = rgb.shape[:2]
    view = np.full((fh+36+24*len(detections), max(fw,640),3),28,np.uint8)
    view[:fh,:fw] = bgr
    for d in detections:
        m = d['metrics']
        cv2.drawContours(view, [d['contour']], -1, (255, 0, 255), 1)
        x, y, _, _ = m['bbox_xywh']
        cv2.polylines(view,[np.rint(d['box']).astype(np.int32)],True,(0,255,255),1)
        label = '#{} {}'.format(m.get('track_id',m['id']), m['decision'])
        cv2.putText(view, label, (x, max(16, y-4)), cv2.FONT_HERSHEY_SIMPLEX, .4, (0,0,0), 3)
        cv2.putText(view, label, (x, max(16, y-4)), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1)
        text = '#{} A={} box={:.3f} model={} gap={:.2f}'.format(
            m.get('track_id',m['id']),m['model_area_pixels'],m['oriented_w_over_h'],
            m['model_label'],m['model_margin'])
        cv2.putText(view,text,(8,fh+24*m['id']),cv2.FONT_HERSHEY_SIMPLEX,.46,(240,240,240),1)
    return view


def write_png(path, pixels):
    ok, encoded = cv2.imencode('.png', pixels)
    if not ok:
        raise OSError('Could not encode PNG')
    encoded.tofile(str(path))


def save_result(output_dir, rgb, out, mask, detections, reference, settings, rules, source=None):
    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_dir / datetime.now().strftime('texture_%Y%m%d_%H%M%S_%f')
    write_png(Path(str(prefix)+'_raw.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix)+'_converted.png'), cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix)+'_mask.png'), mask)
    write_png(Path(str(prefix)+'_removed.png'), process_frame.last_removed_mask)
    write_png(Path(str(prefix)+'_normalized.png'), process_frame.last_normalized_mask)
    write_png(Path(str(prefix)+'_debug.png'), debug_view(out, detections))
    write_png(Path(str(prefix)+'_reference.png'), reference['cutout_bgra'])
    Path(str(prefix)+'.json').write_text(json.dumps({
        'source':source,'settings':asdict(settings),'rules':asdict(rules),
        'model':'102-photo area/rotated-box-ratio RBF; 36 support vectors',
        'timing_ms':process_frame.last_timing,
        'targets': [d['metrics'] for d in detections]}, indent=2), encoding='utf-8')
    print('Saved: {}_*'.format(prefix))
    return prefix


def configure_camera(cam):
    # Exact Recording_N02.py settings/order. No added Picamera2 controls.
    camera = cam.camera
    camera.framerate = 30
    camera.exposure_mode = 'off'
    camera.awb_mode = 'off'
    camera.iso = 100
    camera.shutter_speed = 15000
    camera.exposure_compensation = 0
    camera.awb_gains = (1.2, 1.5)


class LivePreview:
    ORIGINAL = 'Original 640x480'
    CONVERTED = 'Standard height GREEN | S save | R reference | D debug | C reset | Q exit'
    DEBUG = 'Shape debug'
    MASK = 'Stable red foreground mask'

    def __init__(self, args, reference, settings, rules):
        self.args, self.reference, self.settings, self.rules = args, reference, settings, rules
        self.show_debug = args.debug
        self.tracker = Stabilizer(rules)
        self.started = self.last_print = time.monotonic()
        self.frame_times = []
        for title in (self.ORIGINAL, self.CONVERTED):
            cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)

    def run(self, rgb):
        now = time.monotonic()
        if rgb is None:
            if now-self.started > 10:
                raise RuntimeError('No camera image after 10 seconds')
            return
        out, mask, detections = process_frame(rgb,self.reference,self.settings,self.rules,self.tracker)
        self.frame_times.append(process_frame.last_timing['frame_processing_ms'])
        self.frame_times = self.frame_times[-120:]
        cv2.imshow(self.ORIGINAL, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imshow(self.CONVERTED, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
        if self.show_debug:
            cv2.imshow(self.DEBUG, debug_view(out, detections))
            cv2.imshow(self.MASK,self.tracker.last_mask)
        if now-self.last_print >= 1:
            print(' | '.join('#{} A={} box={:.3f} model={} gap={:.2f} state={} votes={}'.format(
                d['metrics'].get('track_id',d['metrics']['id']),
                d['metrics']['model_area_pixels'],d['metrics']['oriented_w_over_h'],
                d['metrics']['model_label'],d['metrics']['model_margin'],d['metrics']['decision'],
                d['metrics']['pending_votes']) for d in detections)
                or 'No red target')
            print('Processing: current={:.1f} ms, median={:.1f} ms, p95={:.1f} ms (last {} frames; capture/display/save excluded)'.format(
                self.frame_times[-1],np.median(self.frame_times),np.percentile(self.frame_times,95),len(self.frame_times)))
            self.last_print = now
        key = cv2.waitKey(1) & 255
        if key in (ord('s'), ord('S')):
            try:
                save_result(self.args.output_dir, rgb, out, mask, detections,
                            self.reference, self.settings, self.rules)
            except (OSError, cv2.error) as exc:
                print('Save failed: {}'.format(exc), file=sys.stderr)
        elif key in (ord('r'), ord('R')):
            try:
                self.reference = standard_reference(sample_reference(rgb))  # Original real green palette; standard template geometry.
                print('Green reference updated from ORIGINAL frame; held until next R.')
            except ValueError as exc:
                print(str(exc))
        elif key in (ord('d'), ord('D')):
            if self.show_debug:
                cv2.destroyWindow(self.DEBUG)
                cv2.destroyWindow(self.MASK)
            self.show_debug = not self.show_debug
        elif key in (ord('c'),ord('C')):
            self.tracker.reset()
            print('Tracking and class memory cleared')
        elif key in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        if cv2.getWindowProperty(self.CONVERTED, cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt

    def shutdown(self):
        cv2.destroyAllWindows()


def read_rgb(path):
    bgr = cv2.imdecode(np.fromfile(str(Path(path).expanduser()), np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise OSError('Cannot read image: {}'.format(path))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def build_parser(default_rules=None):
    defaults = default_rules if default_rules is not None else Rules()
    color_defaults = Settings()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='Photo test instead of camera')
    parser.add_argument('--reference-image', type=Path, help='Photo containing a real green reference pillar')
    parser.add_argument('--no-window', action='store_true', help='Photo test only')
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent/'captures')
    parser.add_argument('--model-margin',type=float,default=defaults.model_margin,help='Model decision margin, not probability')
    parser.add_argument('--edge-pixels',type=int,default=defaults.edge_pixels,help='Dark red edge search radius, 0..6')
    parser.add_argument('--edge-s-min',type=int,default=defaults.edge_s_min)
    parser.add_argument('--edge-v-min',type=int,default=defaults.edge_v_min)
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--switch-frames',type=int,default=defaults.switch_frames,help='Consecutive votes to change a tracked class')
    parser.add_argument('--weak-s-min',type=int,default=color_defaults.weak_s_min,help='Connected dark-red saturation minimum')
    parser.add_argument('--weak-v-min',type=int,default=color_defaults.weak_v_min,help='Connected dark-red brightness minimum')
    parser.add_argument('--grow-pixels',type=int,default=color_defaults.grow_pixels,help='Bounded dark-red growth around current strong seeds')
    parser.add_argument('--tall-scale',type=float,default=defaults.tall_scale)
    parser.add_argument('--low-scale',type=float,default=defaults.low_scale)
    parser.add_argument('--background',choices=['local_white','white'],default=defaults.background)
    parser.add_argument('--benchmark',type=int,default=0,help='Photo mode: timed repetitions after 5 warmup frames')
    return parser


def main():
    parser=build_parser()
    args = parser.parse_args()
    if not 0<=args.model_margin<=3:parser.error('model-margin: 0..3')
    if not 0 <= args.edge_pixels <= 6 or not 0 <= args.edge_s_min <= 255 or not 0 <= args.edge_v_min <= 255:
        parser.error('edge-pixels: 0..6; edge-s-min/edge-v-min: 0..255')
    if not 1<=args.switch_frames<=30:parser.error('switch-frames: 1..30')
    if not 0 <= args.weak_s_min <= 255 or not 0 <= args.weak_v_min <= 255:
        parser.error('weak-s-min/weak-v-min: 0..255')
    if not 0 <= args.grow_pixels <= 12:parser.error('grow-pixels: 0..12')
    if not 0.1<=args.tall_scale<=1 or not 1<=args.low_scale<=4:parser.error('tall-scale: 0.1..1; low-scale: 1..4')
    if not 0<=args.benchmark<=10000:parser.error('benchmark: 0..10000')
    if args.benchmark and not args.image:parser.error('--benchmark requires --image')
    settings = Settings(weak_s_min=args.weak_s_min,weak_v_min=args.weak_v_min,grow_pixels=args.grow_pixels)
    rules = Rules(model_margin=args.model_margin,
                  edge_pixels=args.edge_pixels, edge_s_min=args.edge_s_min, edge_v_min=args.edge_v_min,
                  switch_frames=args.switch_frames,tall_scale=args.tall_scale,low_scale=args.low_scale,background=args.background)
    print('Running file: {}'.format(Path(__file__).resolve()))
    print('Program arguments: {}'.format(sys.argv[1:]))
    print('ACTIVE: learned AREA+BOX model, model_margin={:.3f}, switch_frames={}'.format(
        rules.model_margin,rules.switch_frames))
    print('No normal_min/normal_max ratio thresholds are used in this version.')
    reference = sample_reference(read_rgb(args.reference_image)) if args.reference_image else built_in_reference()
    reference = standard_reference(reference)
    print('HEIGHT: tall_scale={:.4f}, low_scale={:.4f}, background={}'.format(rules.tall_scale,rules.low_scale,rules.background))
    print('Reference: {}'.format(args.reference_image or 'embedded real green pillar from approved photo'))
    if args.image:
        rgb = read_rgb(args.image)
        tracker=Stabilizer(rules)
        out,mask,ds = process_frame(rgb,reference,settings,rules,tracker)
        if args.benchmark:
            for _ in range(5):process_frame(rgb,reference,settings,rules,tracker)
            measurements=[]
            for _ in range(args.benchmark):
                out,mask,ds=process_frame(rgb,reference,settings,rules,tracker)
                measurements.append(process_frame.last_timing.copy())
            report={name:{'mean':float(np.mean([m[name] for m in measurements])),
                'median':float(np.median([m[name] for m in measurements])),
                'p95':float(np.percentile([m[name] for m in measurements],95))}
                for name in measurements[0]}
            report.update(frames=args.benchmark,platform=sys.platform,opencv=cv2.__version__,
                image_shape=list(rgb.shape),capture_display_disk_excluded=True)
            args.output_dir.mkdir(parents=True,exist_ok=True)
            (args.output_dir/'benchmark.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print('BENCHMARK '+json.dumps(report))
        save_result(args.output_dir, rgb, out, mask, ds, reference, settings, rules, str(args.image))
        if not args.no_window:
            try:
                cv2.imshow('Photo result', cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
                cv2.waitKey(0)
            finally:
                cv2.destroyAllWindows()
        return
    if args.no_window:
        parser.error('--no-window is only available with --image')
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        parser.error('Use Raspberry Pi desktop/VNC for live preview; DISPLAY is not set')
    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    car = dk.vehicle.Vehicle()
    cam = PiCamera(image_w=640, image_h=480, image_d=3)
    try:
        time.sleep(2)
        configure_camera(cam)
        preview = LivePreview(args, reference, settings, rules)
    except BaseException:
        cam.shutdown()
        raise
    car.add(cam, outputs=['cam/image_array'], threaded=True)
    car.add(preview, inputs=['cam/image_array'], outputs=[])
    car.start(rate_hz=20)


if __name__ == '__main__':
    def stop(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError, ImportError, cv2.error) as exc:
        print('ERROR: {}'.format(exc), file=sys.stderr)
        sys.exit(1)
