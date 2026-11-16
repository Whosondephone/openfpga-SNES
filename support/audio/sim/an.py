import numpy as np
def load(p):
    v=[]
    for ln in open(p):
        try: v.append(float(ln.split()[0]))
        except: v.append(0.0)
    return np.array(v)
def bh(N):
    n=np.arange(N); a=[0.35875,0.48829,0.14128,0.01168]
    return a[0]-a[1]*np.cos(2*np.pi*n/(N-1))+a[2]*np.cos(4*np.pi*n/(N-1))-a[3]*np.cos(6*np.pi*n/(N-1))
fs_out = 48000*((1e12/13468)/74.25e6)
clk_ratio = (1e12/46560)/21477270
for fr in [440,1000,3000,7000,12000]:
    row=[]
    for nw in [0,1]:
        d=load(f'r_{nw}_{fr}/out.txt')[2000:]
        N=len(d); w=bh(N)
        X=np.abs(np.fft.rfft((d-d.mean())*w))
        k=np.argmax(X); a,b,c=np.log(X[k-1:k+2]+1e-12); p=0.5*(a-c)/(a-2*b+c)
        fpk=(k+p)*fs_out/N
        sig=np.sum(X[k-8:k+9]**2); tot=np.sum(X[3:]**2)
        row.append(f"{'NEW' if nw else 'OLD'}: {fpk:9.2f} Hz, THD+N {10*np.log10((tot-sig)/sig):6.1f} dB")
    print(f"{fr:5d} Hz tone (expected {fr*clk_ratio:9.2f}) | "+" | ".join(row))
