package gzhu.yh;


import gzhu.yh.graphsModel.GenerateGraph;
import gzhu.yh.graphsModel.Graph;
import gzhu.yh.model.Roman12Domination.*;
import gzhu.yh.model.independentRoman2Domination.ApproximationAlgorithm_IR2D;

import java.util.Map;
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
import gzhu.yh.model.maximalRomanDomination.ILP_MDRD_New;
import org.junit.jupiter.api.Test;

import static javax.swing.text.html.HTML.Tag.HEAD;

/**
 * @author wendao
 * @since 2024-09-10
 **/
public class Main {
    public static void main(String[] args) {
        //生成图

        int vertexNum =6;
        //vertexNum = gen.nextInt(20); // TODO 请设置顶点数量
        System.out.println("目前支持的graph类型有："+GenerateGraph.getGraphType());
        //BIPARTITE_GRAPH; BLOCK_GRAPH; COMMON_GRAPH; GRID_GRAPH; INTERVAL_GRAPH; TREE_GRAPH;
//        Graph graph= GenerateGraph.generateGraph("BIPARTITE_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("BLOCK_GRAPH",vertexNum);


     //   Graph graph= GenerateGraph.generateGraph("GRID_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("INTERVAL_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("TREE_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("COMMON_GRAPH",vertexNum);

//        Map<Integer, Integer> integerIntegerMap = ApproximationAlgorithm_IR2D.roman12Domination(graph);

//        Graph graph= GenerateGraph.generateGraph("COMMON_GRAPH",vertexNu
//        m);
//        Graph graph= GenerateGraph.generateGraph("GRID_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("INTERVAL_GRAPH",vertexNum);
//        Graph graph= GenerateGraph.generateGraph("TREE_GRAPH",vertexNum);


        //绘制所生成的图
//        DrawingGraph.visualizeGraphGraphStream(graph);

        //调用gurobi
//        ILP_MDRD_Modify.ILP_MDRD(graph);
//        ILP_MDRD_DrawingOnGraph.ILP_MDRD_DrawingOnGraph(graph);

 //       ILP_RD12_Modify_DrawingOnGraph.ILP_MDRD_DrawingOnGraph(graph);
//        ILP_RD12_Modify_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);

        for (int i =20; i <= 50; i++) {
            Graph graph= GenerateGraph.generateGraph("COMMON_GRAPH",i);
//            ILP_RD12_Approximation_Compare_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);
            ILP_MDRD_New.ILP_MDRD_Apprximate(graph);
        }


//        ILP_RD12_Approximation_Compare_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);

        //输出图的边集
/*        graph.getEdges().forEach(
                edge -> System.out.println("("+edge.getFirst() + "," + edge.getSecond()+ ")")
        );*/
        //输出结果


//        ILP_MDRD_New.ILP_MDRD_DrawingOnGraph(graph);
//        //输出图的边集
//        graph.getEdges().forEach(
//                edge -> System.out.println("("+edge.getFirst() + "," + edge.getSecond()+ ")")
//        );


    }

}
