# Summary

# Completed 
| S.No | Work | Result/Conlusions |
|:-----|:-|:--|
| 1  | Replaced DFS with BFS in seqMDS | 
| 2  | Constructing MST in each iteration of super loop and taking only route (first route) from DFS order. To test whether our intution that taking edges that weight is minimim matters. Performed two different methods |
| 3 | Worked on rajeshParMDS.cpp, conducted testing, and raised concerns. Critical anlaysis for both performance and accuracy of parMDS.cpp vs seqMDS.cpp|
| 4 | Improved the existing seqMDS code, by i) making DFS iterative ii) avodiing unesseary copy of data structures like vectors. Got some benefit in time. Discarded this direction and writing my own code. |
| 5 | Constructed auxilar graph G: <br/> i) D-regular construction, where for each vertex I took closest D points and made it as out-neighbours <br/> ii) Inlcuded MST edges after step 2 for making sure connectivity <br/> Also tried other graph constructions like making edges undirected in above construiction | 



# Ongoing 
- Cone stuff
- Implementing on GPU using CUDA of seqMDS
    - different methods of iterating