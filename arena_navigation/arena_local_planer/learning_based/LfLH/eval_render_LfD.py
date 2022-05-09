import subprocess

cmds = ["/home/users/zizhaowang/hallucination/LfH/eval.py",
        "/home/users/zizhaowang/hallucination/LfD_3D/render_demo_3D.py",
        "/home/users/zizhaowang/hallucination/LfD_3D/LfD_main.py"]
for i, cmd in enumerate(cmds):
    p = subprocess.Popen(["python", "-u", cmd])
    if i < 1:
        try:
            outs, errs = p.communicate(timeout=1800)
        except subprocess.TimeoutExpired:
            p.kill()
            outs, errs = p.communicate()
    else:
        p.communicate()
