package gzhu.yh.graphsModel;

import gzhu.yh.util.Pair;
import gzhu.yh.util.TwoDArrayList;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;


/**
 * @author wendao
 *
 **/


public class ThresholdGraph extends Graph {

    private Double threshold;  // 阈值
    private List<Double> vertexWeights;  // 顶点权重

    // 构造函数
    public ThresholdGraph(Integer v, Integer e, List<List<Integer>> adjMatrix, List<List<Integer>> adjList, List<Pair<Integer, Integer>> edges, Double threshold, List<Double> vertexWeights) {
        super(v, e, adjMatrix, edges);
        List<Integer> vertices=new ArrayList<>();
        for (int i = 0; i < v; i++) {
            vertices.add(i);
        }
        this.setVertices(vertices);
        super.setAdjList(adjList);
        this.threshold = threshold;
        this.vertexWeights = vertexWeights;
        setGraphType();
    }

    @Override
    public void setGraphType() {
        this.setGraphType("THRESHOLD_GRAPH");
    }

    public Double getThreshold() {
        return threshold;
    }

    public void setThreshold(Double threshold) {
        this.threshold = threshold;
    }

    public List<Double> getVertexWeights() {
        return vertexWeights;
    }

    public void setVertexWeights(List<Double> vertexWeights) {
        this.vertexWeights = vertexWeights;
    }

    // 静态方法：生成随机的阈值图
    public static ThresholdGraph randomGenTresholdGraphByVertexNum(Integer vertexNum) {
        Random rand = new Random();

        // 随机生成顶点的权重
        List<Double> vertexWeights = new ArrayList<>();
        for (int i = 0; i < vertexNum; i++) {
            vertexWeights.add(rand.nextDouble() * 100);  // 生成一个0到100之间的随机浮动权重
        }

        // 随机生成阈值，通常设定为所有权重的平均值
        Double threshold = vertexWeights.stream().mapToDouble(Double::doubleValue).average().orElse(50.0);

        // 初始化邻接矩阵和邻接表
        List<List<Integer>> adjMatrix = TwoDArrayList.createTwoDArrayList(vertexNum, vertexNum,0);
        List<List<Integer>> adjList = TwoDArrayList.createTwoDArrayList(vertexNum);
        List<Pair<Integer, Integer>> edges = new ArrayList<>();

        // 遍历每一对顶点，检查是否应该添加边
        for (int i = 0; i < vertexNum; i++) {
            //无向图，所以是j=i+1
            for (int j = i + 1; j < vertexNum; j++) {
                Double weightSum = vertexWeights.get(i) + vertexWeights.get(j);
                if (weightSum > threshold) {
                    // 添加边到邻接矩阵和邻接表
                    //邻接矩阵,set()替换
                    adjMatrix.get(i).set(j, 1);// i 和 j 相邻
                    adjMatrix.get(j).set(i, 1);// j 和 i 相邻
                    //邻接表，add()追加
                    adjList.get(i).add(j);    // i 的邻接点
                    adjList.get(j).add(i);    // j 的邻接点
                    //往边的列表记录
                    edges.add(new Pair<Integer,Integer>(i, j));
                }
//                else {// 添加0到邻接矩阵（没有边）
//                }
            }
        }

        // 返回生成的阈值图
        return new ThresholdGraph(vertexNum, edges.size(), adjMatrix, adjList, edges, threshold, vertexWeights);
    }
}
