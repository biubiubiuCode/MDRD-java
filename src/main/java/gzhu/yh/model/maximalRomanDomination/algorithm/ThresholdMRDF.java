package gzhu.yh.model.maximalRomanDomination.algorithm;

import gzhu.yh.annotation.LogExecutionTime;
import gzhu.yh.graphsModel.Graph;
import org.springframework.stereotype.Service;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

/**
 * @author wendao
 **/
@Service
public class ThresholdMRDF {
    @LogExecutionTime
    public int ThresholdSolve(Graph g) {

        int vertexNum=g.getV();
        boolean[] visited = new boolean[vertexNum];
        int minDegree = Integer.MAX_VALUE;

        List<List<Integer>> adjList = g.getAdjList();
        int edgeCount = 0; // 用于判断完全图

        for (int i = 0; i < vertexNum; i++) {
            if (!visited[i]) {
                Queue<Integer> queue = new LinkedList<>();
                queue.add(i);
                visited[i] = true;

                while (!queue.isEmpty()) {
                    int curr = queue.poll();
                    List<Integer> neighbors = adjList.get(curr);
                    int degree = neighbors.size();
                    minDegree = Math.min(minDegree, degree);//统计最小度
                    edgeCount += degree;

                    for (int neighbor : neighbors) {
                        if (!visited[neighbor]) {
                            visited[neighbor] = true;
                            queue.add(neighbor);
                        }
                    }
                }
            }
        }

        // 因为无向图中每条边被统计了两次
        edgeCount /= 2;

        // 判断是否是完全图
        boolean isComplete = edgeCount == vertexNum * (vertexNum - 1) / 2;
        System.out.print("图是完全图？" + isComplete + " \t ");
        if (isComplete){
            return minDegree+1;
        } else {
            return minDegree+2;
        }
    }

}
