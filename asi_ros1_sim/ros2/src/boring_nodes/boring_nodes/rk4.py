def rk4(x,y,h,derivs):
  n = len(y)
  k1 = derivs(x, y)
  ym = [y[ii] + k1[ii]*h/2.0 for ii in range(n)]
  k2 = derivs(x+h/2.0, ym)
  ym = [y[ii] + k2[ii]*h/2.0 for ii in range(n)]
  k3 = derivs(x+h/2.0, ym)
  ye = [y[ii] + k3[ii]*h for ii in range(n)]
  k4 = derivs(x+h, ye)
  return [y[ii] + ((k1[ii] + 2.0*(k2[ii]+k3[ii]) + k4[ii])/6.0)*h for ii in range(n)]
