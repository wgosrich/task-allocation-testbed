import networkx as nx
from networkx.algorithms import bipartite
import numpy as np
import scipy
from networkx.algorithms import matching
import math

class CentralizedHungarianPlanner():

    def __init__(self,env):
        self.env = env
    def plan(self):
        agent_state = self.env.x
        task_state = self.env.tasks
        print(agent_state, 'agent state')
        print(task_state, 'task state')

        n=len(agent_state)
        wt_array=np.zeros((n,n))
        for i in range(0,n):
          for j in range(0,n):
            wt_array[i][j] = math.sqrt((agent_state[i][0] - task_state[j][0])**2 + (agent_state[i][1] - task_state[j][1])**2)
        
        Inf = float('inf')
        wts=scipy.sparse.lil_matrix(wt_array) #types of sparse matrix? #is Inf okay
        B=bipartite.matrix.from_biadjacency_matrix(wts)

        #node labels
        left_nodes, right_nodes = bipartite.sets(B)
        l_wts = {node: min(wts.toarray()[i]) for i, node in enumerate(left_nodes)} 
        r_wts = {node: 0 for node in right_nodes}

        #equality subgraph
        eq_subgraph = nx.Graph()
        eq_subgraph.add_nodes_from(left_nodes, bipartite=0)
        eq_subgraph.add_nodes_from(right_nodes, bipartite=1)
        edges = ((edge[0], edge[1], {'weight': edge[2]['weight']})
                for edge in B.edges(data=True)  #with wieghts
                if (l_wts[edge[0]] + r_wts[edge[1]] == edge[2]['weight'])) 

        # eq_subgraph.update(edges)
        eq_subgraph.add_edges_from(edges)

        # maximum matching
        Max_match = bipartite.matching.hopcroft_karp_matching(eq_subgraph, top_nodes=left_nodes)

        #Min vertex cover
        Min_vertex = bipartite.matching.to_vertex_cover(eq_subgraph, Max_match, top_nodes=left_nodes)

        while(not matching.is_perfect_matching(B,Max_match)):
          
          Rc = left_nodes.intersection(Min_vertex)
          Pc = right_nodes.intersection(Min_vertex)
          Rc_not = left_nodes.difference(Rc)
          Pc_not = right_nodes.difference(Pc)
          E_cand={}

          # Step 1(a):
          for i in Rc_not:
            min_slack = Inf
            for j in Pc_not:
              j_n = j-n
              if wt_array[i][j_n] != Inf:
                slack = wt_array[i][j_n]-l_wts[i]-r_wts[j]
                if slack < min_slack:
                  min_slack = slack
                  min_j = j
                E_cand[i] = min_j

          # Step 1(b):
          min_slack = Inf
          for i in E_cand.keys():
            j_n=E_cand[i]-n
            j=E_cand[i]
            slack = wt_array[i][j_n]-l_wts[i]-r_wts[j]
            if slack < min_slack:
              min_slack = slack
              i_new = i
          delta_slack = min_slack
          if i_new in Rc:
            l_wts[i_new]= l_wts[i_new] - delta_slack
          if E_cand[i_new] in Pc_not:
            r_wts[E_cand[i_new]]= r_wts[E_cand[i_new]] + delta_slack

          # Step 2:
          # new_edge
          eq_subgraph.add_edge(i_new, E_cand[i_new], weight= wt_array[i_new][E_cand[i_new]-n])
          # maximum matching
          Max_match = bipartite.matching.hopcroft_karp_matching(eq_subgraph, top_nodes=left_nodes)
          print(Max_match, 'Maximum matching')
          #Min vertex cover
          Min_vertex = bipartite.matching.to_vertex_cover(eq_subgraph, Max_match, top_nodes=left_nodes)

        m=list(Max_match.values())
        l = [[x-n] for x in m]
        print(l[0:n], 'Allocation list ret')
        return l[0:n]
