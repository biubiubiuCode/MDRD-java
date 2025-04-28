package gzhu.yh.model.maximalRomanDomination.algorithm;

/**
 * @author wendao
 **/
import gzhu.yh.graphsModel.Graph;

import java.io.*;
import java.util.*;

/**
 * Greedy heuristic for Maximal Roman Domination (MRDF) on a simple undirected graph.
 *
 * This is an approximation (heuristic) algorithm without formal ratio guarantee,
 * aiming to produce a low-weight MRDF assignment in reasonable time.
 *
 * Usage:
 *   Read a graph into a GreedyMRDF.Graph, then call greedySolve() to get f: V-> {0,1,2}.
 */
public class GreedyMRDF {

    /**
     * Computes an approximate MRDF assignment.
     * @return int[] f of length n, f[v] in {0,1,2}.
     * 初始化 所有顶点为 0，并计算哪些“0”未被 2 覆盖。
     *
     * 贪心覆盖：不断选取一个未覆盖的 0-顶点，再在其邻居中挑选能覆盖最多零点的顶点，将其设为 2，直到所有 0 点都被覆盖。
     *
     * 保证自立性：检查是否已有一个f(v)=1
     *   若f(v)>=1(实际上必为2) 且其邻居均非 0 的顶点，则已经满足；若没有，则选一个最小度顶点设为 1，并对其邻居中仍为 0 的点全部赋值为 1。
     */
    public static int[] greedySolve(Graph g) {
        int n = g.getV();
        int[] f = new int[n];             // f[v] = 0,1,2 assignment
        boolean[] covered = new boolean[n];

        // Initially all f[v]=0, none covered
        Arrays.fill(f, 0);
        Arrays.fill(covered, false);

        // Helper to update coverage of zeros
        Runnable updateCoverage = () -> {
            boolean[] has2neighbor = new boolean[n];
            for (int v = 0; v < n; v++) {
                if (f[v] == 2) {
                    for (int w : g.getAdjList().get(v)) has2neighbor[w] = true;
                }
            }
            // 仅对 f[v]==0 的顶点，根据 has2neighbor 更新 covered[v]
            for (int v = 0; v < n; v++) {
                covered[v] = !(f[v] == 0 && !has2neighbor[v]);
//                if(f[v]==0) covered[v]=has2neighbor[v];
            }
        };

        // Greedy: cover all zeros
        updateCoverage.run();
        while (true) {
            int bestV = -1, bestScore = -1; //bestV is the index of v in array f[]
            // Find uncovered zero vertex
            for (int v = 0; v < n; v++) {
                if (f[v] == 0 && !covered[v]) {
                    bestV = v;
                    break;
                }
            }
            if (bestV < 0) break; //all vertex are covered

            // Among its neighbors, choose w to maximize number of newly covered zeros
            int pick = -1, maxNew = -1; // pick is the index of w
            for (int w : g.getAdjList().get(bestV)) {
                int newCover = 0;
                for (int u : g.getAdjList().get(w)) {
                    if (f[u] == 0 && !covered[u]) newCover++;
                }
                if (newCover > maxNew) {
                    maxNew = newCover;
                    pick = w;
                }
            }
            if (pick < 0) {
                // no neighbor available: fallback to set bestV itself to 2
                pick = bestV;
            }
            f[pick] = 2;
            updateCoverage.run();
        }

        // Ensure MRDF condition (b): at least one "self-reliant" node in V1
        boolean hasSelf = false;
        for (int u = 0; u < n; u++) {
            if (f[u] >= 1) {
                boolean ok = true;
                for (int w : g.getAdjList().get(u)) {
                    if (f[w] == 0) { ok = false; break; }
                }
                if (ok) { hasSelf = true; break; }
            }
        }
        if (!hasSelf) {
            // pick a vertex of minimum degree to serve as self-reliant
            int uStar = 0; // assume v_0 is the vertex with minium degree
            for (int v = 1; v < n; v++) {
                if (g.getAdjList().get(v).size() < g.getAdjList().get(uStar).size()) uStar = v;
            }
            // assign it to 1
            f[uStar] = 1;
            // cover its neighbors by setting any zero neighbor to 1
            for (int w : g.getAdjList().get(uStar)) {
                if (f[w] == 0) f[w] = 1;
            }
        }
        return f;
    }

    // Example usage with file input
//    public static void main(String[] args) throws IOException {
//        if (args.length < 1) {
//            System.err.println("Usage: java GreedyMRDF <graph_file>");
//            return;
//        }
//        BufferedReader br = new BufferedReader(new FileReader(args[0]));
//        String[] hd = br.readLine().split("\\s+");
//        int n = Integer.parseInt(hd[0]);
//        int m = Integer.parseInt(hd[1]);
//        Graph g = new Graph(n);
//        for (int i = 0; i < m; i++) {
//            String[] e = br.readLine().split("\\s+");
//            int u = Integer.parseInt(e[0]), v = Integer.parseInt(e[1]);
//            g.addEdge(u, v);
//        }
//        br.close();
//
//        int[] sol = greedySolve(g);
//        int weight = 0;
//        for (int v = 0; v < n; v++) {
//            weight += sol[v];
//            System.out.printf("v=%d: f=%d\n", v, sol[v]);
//        }
//        System.out.println("Total weight = " + weight);
//    }
}
