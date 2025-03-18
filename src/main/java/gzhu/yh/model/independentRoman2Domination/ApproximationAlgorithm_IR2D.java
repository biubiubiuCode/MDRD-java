package gzhu.yh.model.independentRoman2Domination;
import java.util.*;
import gzhu.yh.graphsModel.Graph;

public class  ApproximationAlgorithm_IR2D{

    public static Map<Integer, Integer> roman12Domination(Graph graph) {
        int n = graph.getV(); // 获取顶点数量
        List<List<Integer>> adjList = graph.getAdjList(); // 获取邻接表

        // 初始化顶点赋值
        Map<Integer, Integer> f = new HashMap<>();
        Map<Integer, Integer> x = new HashMap<>();
        Map<Integer, Integer> y = new HashMap<>();
        Map<Integer, Integer> z = new HashMap<>();
        Map<Integer, Double> alpha = new HashMap<>();
        Map<Integer, Double> beta = new HashMap<>();

        Set<Integer> U = new HashSet<>(); // 未满足约束的顶点集合
        for (Integer i = 0; i < graph.getV(); i++) {
            U.add(i);
        }
        for (Integer v = 0; v < graph.getV(); v++) {
            f.put(v, 0);
            x.put(v, 0);
            y.put(v, 0);
            z.put(v, 1);
            alpha.put(v, 0.0);
            beta.put(v, 0.0);
        }

        double epsilon = 1e-6;
        double delta;

        while (!U.isEmpty()) {
            // ===== 动态步长调整 =====
            for (int v : U) {
                int sumNeighbors = 0;
                for (int u : adjList.get(v)) {
                    sumNeighbors += x.get(u);
                }

                if (sumNeighbors < 1) {
                    // 公式 1: 动态步长调整 (α_v)
                    delta = calculateDelta(graph, adjList, alpha, beta, v);
                    alpha.put(v, alpha.get(v) + delta);
                }

                if (sumNeighbors > 2) {
                    // 公式 2: 动态步长调整 (β_v)
                    delta = calculateDelta(graph, adjList, beta, alpha, v);
                    beta.put(v, beta.get(v) + delta);
                }
            }

            // ===== 触发条件判断 =====
            boolean triggered = false;
            for (Integer u = 0; u < graph.getV(); u++) {
                double dualSum = 0;
                for (int v : adjList.get(u)) {
                    dualSum += alpha.get(v) - beta.get(v);
                }
                if (dualSum >= 2 - epsilon) {
                    // 触发，将该节点设置为 2
                    f.put(u, 2);
                    x.put(u, 1);
                    y.put(u, 0);
                    z.put(u, 0);
                    triggered = true;

                    // 更新邻居状态
                    for (int v : adjList.get(u)) {
                        int s_v = 0;
                        for (int w : adjList.get(v)) {
                            s_v += x.get(w);
                        }

                        if (s_v >= 1 && s_v <= 2) {
                            U.remove(v);
                        } else if (s_v > 2) {
                            // 上界违反：设置 v 为 1
                            f.put(v, 1);
                            y.put(v, 1);
                            x.put(v, 0);
                            z.put(v, 0);
                            U.remove(v);
                        }
                    }
                }
            }

            if (!triggered) {
                // 如果没有触发，可以适当增加步长（用动态方式替代固定值）
                delta = Math.min(0.1, calculateMinDelta(graph, adjList, alpha, beta));
            }

        /*double delta = 0.1;

        while (!U.isEmpty()) {
            // Step 1: 对偶变量增长
            for (int v : U) {
                int s_v = 0;
                for (int u : adjList.get(v)) {
                    s_v += x.get(u);
                }

                if (s_v < 1) {
                    alpha.put(v, alpha.get(v) + delta);
                }
                if (s_v > 2) {
                    beta.put(v, beta.get(v) + delta);
                }
            }
        */

           /* // Step 2: 检查触发条件
            boolean triggered = false;
//            for (int u : graph.getVertices()) {
            for (Integer u = 0; u < graph.getV(); u++) {
                double dualSum = 0;
                for (int v : adjList.get(u)) {
                    dualSum += alpha.get(v) - beta.get(v);
                }
                if (dualSum >= 2 - epsilon) {
                    // 触发，将该节点设置为 2
                    f.put(u, 2);
                    x.put(u, 1);
                    y.put(u, 0);
                    z.put(u, 0);
                    triggered = true;

                    // 更新邻居状态
                    for (int v : adjList.get(u)) {
                        int s_v = 0;
                        for (int w : adjList.get(v)) {
                            s_v += x.get(w);
                        }

                        if (s_v >= 1 && s_v <= 2) {
                            U.remove(v);
                        } else if (s_v > 2) {
                            // 上界违反：设置 v 为 1
                            f.put(v, 1);
                            y.put(v, 1);
                            x.put(v, 0);
                            z.put(v, 0);
                            U.remove(v);
                        }
                    }
                }
            }

            if (!triggered) {
                // 如果没有触发，将步长增加

                delta *= 1.5;
            }*/
        }

//        // 计算总成本
//        int totalCost = f.values().stream().mapToInt(Integer::intValue).sum();
//
//        System.out.println("近似算法顶点赋值: " + f);
//        System.out.println("总成本: " + totalCost);

        return f;
    }

    private static double calculateDelta(Graph graph, List<List<Integer>> adjList,
                                         Map<Integer, Double> alpha,
                                         Map<Integer, Double> beta,
                                         int v) {
        double minDelta = Double.MAX_VALUE;
        for (int u : adjList.get(v)) {
            double degree = adjList.get(u).size(); // u 的度数
            double sum = 0;
            for (int w : adjList.get(u)) {
                sum += (alpha.get(w) - beta.get(w));
            }
            double delta = (2 - sum) / degree;
            minDelta = Math.min(minDelta, delta);
        }
        return minDelta;
    }

    // 计算全局最小步长
    private static double calculateMinDelta(Graph graph, List<List<Integer>> adjList,
                                            Map<Integer, Double> alpha,
                                            Map<Integer, Double> beta) {
        double minDelta = Double.MAX_VALUE;
        for (Integer v = 0; v < graph.getV(); v++) {
            double delta = calculateDelta(graph, adjList, alpha, beta, v);
            minDelta = Math.min(minDelta, delta);
        }
        return minDelta;
    }
}

