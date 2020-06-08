import numpy as np
# n=5
Inf = float('inf')
# w = np.array([[5,Inf,Inf,Inf,Inf],[3,7,3,Inf,Inf],[6,Inf,2,100,4],[Inf,Inf,5,6,2],[Inf,4,Inf,1,6]])
#example test
n=4
w = np.array([[30,40,50,60],[70,30,40,70],[60,50,60,30],[20,80,50,70]])
u = np.arange(n)
v = np.arange(n)

U  = V = range(n)
#arbitary feasible label function trivial 
lu=[]
for u in range(n):
  lu.append(min(w[u]))
lv = [0 for v in V]

#Equality subgraph
Ey = np.zeros((n,n))
for u in range(n):
  for v in range(n):
    if lu[u]+lv[v]==w[u][v]:
      Ey[u][v]=w[u][v]

#Maximum cardinality matching
M = np.zeros((n,n))
Rc_not = list(range(n)) #uncovered vertices
Pc_not = list(range(n))
Pc=[]
Rc=[]
for u in range(n):
  for v in Pc_not:
    if Ey[u][v] != 0:
      M[u][v]=Ey[u][v]
      Pc.append(v)  #which vertice to add???
      Pc_not.remove(v)
      break
#Minimum vertex cover
M_vc = Pc+Rc # no differentiation between Vc and Rc vertices.. is that ok?

#check for perfect matching
E_cand = np.zeros((n,n)) #candidate edges
if len(Pc_not)!=0:
  print("Not a perfect matching")
  for i, r in enumerate(Rc_not, 0):
    # sl_R = np.zeros(len(Pc_not))
    min_slack = Inf
    for j, p in enumerate(Pc_not, 0):
      if w[r][p] != Inf:
        slack = w[r][p]-lu[r]-lv[p]
        if slack < min_slack:
          min_slack = slack
          E_cand[r][p]=w[r][p]  #change for sparse matrix #use dict?

  #Step 1b:
  #change for sparse matrix
  for r in range(n):
    min_slack = Inf
    for p in range(n):
      if E_cand[r][p] != 0:
        slack = w[r][p]-lu[r]-lv[p]
        if slack < min_slack:
          del_min_slack = slack
          new_edge_r = r
          new_edge_p = p
  if new_edge_r in Rc:
    lu[new_edge_r]= lu[new_edge_r]-del_min_slack
  if new_edge_p in Pc_not:
    lv[new_edge_p]= lv[new_edge_p]+del_min_slack

  #Step2:
  #Equality subgraph
  Ey = np.zeros((n,n))
  for u in range(n):
    for v in range(n):
      if lu[u]+lv[v]==w[u][v]:
        Ey[u][v]=w[u][v]

  #Maximum cardinality matching
  M = np.zeros((n,n))
  Rc_not = list(range(n)) #uncovered vertices
  Pc_not = list(range(n))
  Pc=[]
  Rc=[]
  for u in range(n):
    for v in Pc_not:
      if Ey[u][v] != 0:
        M[u][v]=Ey[u][v]
        Pc.append(v)  #which vertice to add???
        Pc_not.remove(v)
        break
  #Minimum vertex cover
  M_vc = Pc+Rc # no differentiation between Vc and Rc vertices.. is that ok?
