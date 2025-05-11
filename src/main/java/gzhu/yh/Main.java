package gzhu.yh;


import gzhu.yh.graphsModel.GenerateGraph;
import gzhu.yh.graphsModel.Graph;

import java.util.Arrays;

import gzhu.yh.model.maximalRomanDomination.ILP_MDRD_New;
import gzhu.yh.model.maximalRomanDomination.algorithm.GreedyMRDF;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.EnableAspectJAutoProxy;
import org.springframework.stereotype.Component;


/**
 * @author wendao
 * @since 2024-09-10
 **/
@SpringBootApplication
@EnableAspectJAutoProxy
public class Main implements CommandLineRunner {
    @Autowired
    private GreedyMRDF greedyMRDF;

    public static void main(String[] args) {
        SpringApplication.run(Main.class, args);
    }

    @Override
    public void run(String... args) throws Exception {
        //生成图

        int vertexNum =6;
        //vertexNum = gen.nextInt(20); // TODO 请设置顶点数量
        System.out.println("目前支持的graph类型有："+GenerateGraph.getGraphType());
        //BIPARTITE_GRAPH; BLOCK_GRAPH; COMMON_GRAPH; GRID_GRAPH; INTERVAL_GRAPH; THRESHOLD_GRAPH; TREE_GRAPH;
        String graphType = "THRESHOLD_GRAPH";


        //绘制所生成的图
//        DrawingGraph.visualizeGraphGraphStream(graph);

        //调用gurobi
 //       ILP_RD12_Modify_DrawingOnGraph.ILP_MDRD_DrawingOnGraph(graph);
//        ILP_RD12_Modify_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);

        for (int i =300; i <= 300; i++) {
            Graph graph= GenerateGraph.generateGraph(graphType,i);
//            ILP_RD12_Approximation_Compare_DrawingOnGraph.ILP_RD12_DrawingOnGraph(graph);
//            ILP_MDRD_New.ILP_MDRD_Apprximate(graph);
              //近似算法
            System.out.print("图:"+graphType+" 包含顶点数为" + i+"\t " );
            ILP_MDRD_New.ILP_MDRD_ThresholdGraph(graph);

//            int sum = Arrays.stream(greedyMRDF.greedySolve(graph)).sum();
//            System.out.println("当前顶点数量 " + i + " 的图的近似算法结果总和: " + sum);
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
