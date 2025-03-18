package gzhu.yh;

<<<<<<< HEAD
import gzhu.yh.graphsModel.GenerateGraph;
import gzhu.yh.graphsModel.Graph;
import gzhu.yh.model.Roman12Domination.*;
import gzhu.yh.model.independentRoman2Domination.ApproximationAlgorithm_IR2D;

import java.util.Map;
=======
import com.gurobi.gurobi.GRBException;
import gzhu.yh.graphDrawing.DrawingGraph;
import gzhu.yh.graphsModel.CommonGraph;
import gzhu.yh.graphsModel.GenerateGraph;
import gzhu.yh.graphsModel.Graph;
import gzhu.yh.graphsModel.TreeGraph;
import gzhu.yh.model.maximalRomanDomination.ILP_MDRD;
import gzhu.yh.model.maximalRomanDomination.ILP_MDRD_DrawingOnGraph;
import gzhu.yh.model.maximalRomanDomination.ILP_MDRD_Modify;
import gzhu.yh.model.maximalRomanDomination.ILP_MDRD_Modify_DrawingOnGraph;
import org.junit.jupiter.api.Test;
>>>>>>> 87d90ed54f0d8f9808149bb81fefec4c631bf5c4

/**
 * @author wendao
 * @since 2024-09-10
 **/
public class Main {
    public static void main(String[] args) {
        //生成图
<<<<<<< HEAD
        int vertexNum =50;
=======
        int vertexNum = 20;
>>>>>>> 87d90ed54f0d8f9808149bb81fefec4c631bf5c4
        //vertexNum = gen.nextInt(20); // TODO 请设置顶点数量
        System.out.println("目前支持的graph类型有："+GenerateGraph.getGraphType());
        //BIPARTITE_GRAPH; BLOCK_GRAPH; COMMON_GRAPH; GRID_GRAPH; INTERVAL_GRAPH; TREE_GRAPH;
//        Graph graph= GenerateGraph.generateGraph("BIPARTITE_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("BLOCK_GRAPH",vertexNum);
<<<<<<< HEAD

     //   Graph graph= GenerateGraph.generateGraph("GRID_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("INTERVAL_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("TREE_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("COMMON_GRAPH",vertexNum);

//        Map<Integer, Integer> integerIntegerMap = ApproximationAlgorithm_IR2D.roman12Domination(graph);
=======
//        Graph graph= GenerateGraph.generateGraph("COMMON_GRAPH",vertexNu
//        m);
//        Graph graph= GenerateGraph.generateGraph("GRID_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("INTERVAL_GRAPH",vertexNum);
        Graph graph= GenerateGraph.generateGraph("TREE_GRAPH",vertexNum);

>>>>>>> 87d90ed54f0d8f9808149bb81fefec4c631bf5c4
        //绘制所生成的图
//        DrawingGraph.visualizeGraphGraphStream(graph);

        //调用gurobi
//        ILP_MDRD_Modify.ILP_MDRD(graph);
//        ILP_MDRD_DrawingOnGraph.ILP_MDRD_DrawingOnGraph(graph);
<<<<<<< HEAD
 //       ILP_RD12_Modify_DrawingOnGraph.ILP_MDRD_DrawingOnGraph(graph);
//        ILP_RD12_Modify_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);

        for (int i =20; i <= 100; i++) {
            Graph graph= GenerateGraph.generateGraph("COMMON_GRAPH",i);
            ILP_RD12_Approximation_Compare_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);
        }


//        ILP_RD12_Approximation_Compare_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);

        //输出图的边集
/*        graph.getEdges().forEach(
                edge -> System.out.println("("+edge.getFirst() + "," + edge.getSecond()+ ")")
        );*/
        //输出结果





=======
        ILP_MDRD_Modify_DrawingOnGraph.ILP_MDRD_DrawingOnGraph(graph);
        //输出图的边集
        graph.getEdges().forEach(
                edge -> System.out.println("("+edge.getFirst() + "," + edge.getSecond()+ ")")
        );
>>>>>>> 87d90ed54f0d8f9808149bb81fefec4c631bf5c4

    }

}
