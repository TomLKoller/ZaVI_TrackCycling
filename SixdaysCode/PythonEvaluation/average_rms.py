#Average RMS is calculated as the mean of the 15 RMSs from the barriers


rms=0.19+0.061+0.061+0.054+0.085
rms+=0.164+0.103+0.060+0.078+0.109
rms+=0.18+0.027+0.045+0.06+0.071
print(rms/15.)

sigma=0.128+0.039+0.039+0.038+0.041
sigma+=0.040+0.09+0.034+0.075+0.076
sigma+=0.036+0.021+0.021+0.031+0.016

print(sigma/15.)


old_sigma=0.094+0.07+0.058+0.061+0.047
old_sigma+=0.083+0.107+0.092+0.141+0.068
old_sigma+=0.074+0.037+0.093+0.056+0.080
print(old_sigma/15.)
