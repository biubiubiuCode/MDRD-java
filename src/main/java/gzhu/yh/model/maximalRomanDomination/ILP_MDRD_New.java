package gzhu.yh.model.maximalRomanDomination;

import com.gurobi.gurobi.GRB;
import com.gurobi.gurobi.GRBEnv;
import com.gurobi.gurobi.GRBException;
import com.gurobi.gurobi.GRBLinExpr;
import com.gurobi.gurobi.GRBModel;
import com.gurobi.gurobi.GRBVar;
import gzhu.yh.annotation.LogExecutionTime;
import gzhu.yh.graphsModel.Graph;
import gzhu.yh.model.independentRoman2Domination.ApproximationAlgorithm_IR2D;
import gzhu.yh.model.maximalRomanDomination.algorithm.GreedyMRDF;
import gzhu.yh.util.Pair;
import gzhu.yh.util.SpringContextUtil;
import org.graphstream.graph.implementations.SingleGraph;
import org.graphstream.ui.view.Viewer;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/**
 * @author wendao
 * @since 2025-4-28
 * //
 * <p>
 * maxiaml roman domiation的ILP方程
 *      varialble:
 *              x_{v,1}:顶点是否赋值为1。是则取值1，否取值0
 *              x_{v,2}:顶点是否赋值为2。是则取值1，否取值0
 *              z_v:顶点是否不与赋值为0的点不相连。是则取值0，否取值1
 *      minumum: x_{v,1} + 2 * x_{v,2}
 *      subject to:
给定图 \(G=(V,E)\)，令 \(\deg(u)=|N(u)|\)。定义决策变量
        \[
        \begin{aligned}
            &x_v^{(0)},\,x_v^{(1)},\,x_v^{(2)} \in \{0,1\},\quad \forall v\in V,\\
            &y_u \in \{0,1\},\quad \forall u\in V,
        \end{aligned}
        \]
        其中 \(x_v^{(i)}=1\) 表示 \(f(v)=i\)，\(y_u=1\) 表示顶点 \(u\) 是一个“自立点”。

        目标函数：
        \begin{equation}
            \min\quad
            \sum_{v\in V}\bigl(2\,x_v^{(2)} + 1\,x_v^{(1)}\bigr).
        \end{equation}

        约束条件：
        \begin{align}
            &\forall v\in V:\quad
            x_v^{(0)} + x_v^{(1)} + x_v^{(2)} = 1,
        &\text{（唯一赋值）}
        \\
            &\forall v\in V:\quad
            x_v^{(0)} \le \sum_{w\in N(v)} x_w^{(2)},
        &\text{（0点覆盖）}
        \\
            &\forall u\in V:\quad
            y_u \le x_u^{(1)},
            &\text{（必为 $V_1$）}
        \\
            &\forall u\in V:\quad
            \sum_{w\in N(u)} x_w^{(0)} \;\le\;\bigl(1 - y_u\bigr)\,\deg(u),
            &\text{（邻居无空点）}
        \\
            &\forall u\in V:\quad
            y_u \;\ge\; x_u^{(1)} \;-\; \sum_{w\in N(u)} x_w^{(0)},
            &\text{（完整标记）}
        \\
            &\sum_{u\in V} y_u \;\ge\; 1.
            &\text{（至少一个自立点）}
        \end{align}
 **/
@Service
public class ILP_MDRD_New {
    @Autowired
    private GreedyMRDF greedyMRDF;
    public static void ILP_MDRD_DrawingOnGraph(Graph graph){
        try {
            // 创建环境
            GRBEnv env = new GRBEnv(true);
            env.set("logFile", "src/main/java/gzhu/yh/logger/ILP_MDRD.log"); //设置日志文件
            env.start();

            // 创建模型
            GRBModel model = new GRBModel(env);
            model.set(GRB.StringAttr.ModelName, "ILP_MDRD");
            // 获取图的属性
            int numVertices = graph.getV(); // 顶点数
            List<List<Integer>> adjMatrix = graph.getAdjMatrix(); // 邻接矩阵

            // 定义变量
            GRBVar[][] x = new GRBVar[numVertices][3]; // 0: x_v^0, 1: x_v^1, 2: x_v^2
            GRBVar[] y = new GRBVar[numVertices]; // 辅助变量 y_w, 当前点是否满足极大性约束

            for (int v = 0; v < numVertices; v++) {
                x[v][0] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_0");//赋值为0
                x[v][1] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_1");//赋值为1
                x[v][2] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_2");//赋值为2
                y[v] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "y_" + v);//满足极大性
            }

            // 约束1：唯一赋值
            // x0_v + x1_v + x2_v=1
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr constraint1 = new GRBLinExpr();
                constraint1.addTerm(1.0, x[v][0]);
                constraint1.addTerm(1.0, x[v][1]);
                constraint1.addTerm(1.0, x[v][2]);
                model.addConstr(constraint1, GRB.EQUAL, 1.0,"约束1：唯一赋值");
            }

            // 约束2：罗马约束
            // x0_v <= \sum_{u\in N(v) x2_u}
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr constraint2 = new GRBLinExpr();
                for (int u = 0; u < numVertices; u++) {
                    if (adjMatrix.get(v).get(u) == 1) {
                        constraint2.addTerm(1.0, x[u][2]);
                    }
                }
                model.addConstr(x[v][0], GRB.LESS_EQUAL, constraint2,"约束2:罗马约束");
            }
            //约束合集：极大性约束描述
                // 约束3a：极大性出自V_1
                // y_v <= x1_v  for v in V
                for (int v = 0; v < numVertices; v++) {
                    GRBLinExpr constraint3a = new GRBLinExpr();
                    constraint3a.addTerm(1.0,x[v][1]);
                    model.addConstr(y[v], GRB.LESS_EQUAL, constraint3a,"约束3a：极大性出自V_1");
                }

                // 约束3b：满足极大性则邻点无0
                // sum_{u\in N(v)}(x0_u) <= (1-y_v)deg(v) for v in V
                /*
                    y_v=1时，右侧为0，强制所有邻点x0_u = 0
                    y_v=0时，约束相当于没有
                 */
                for (int v = 0; v < numVertices; v++) {
                    GRBLinExpr constraint3b = new GRBLinExpr();

                    int deg_v=0;//v的度
                    for (int u = 0; u < numVertices; u++) {
                        if (adjMatrix.get(v).get(u) == 1) {
                            constraint3b.addTerm(1.0, x[u][0]);
                            deg_v++;
                        }
                    }
                    constraint3b.addTerm(deg_v,y[v]);
                    model.addConstr(constraint3b, GRB.LESS_EQUAL, deg_v,"约束3b：满足极大性则邻点无0");

                }
                // 约束3c：满足极大性的全部标记y_v=1
                // y_v >= x1_v - sum_{u\in N(v)}(x0_u) for v in V
                    /*
                        当且仅当 x1_v=1且所有邻点x_0u = 0才有y_v=1
                        其他情况需与约束3a、3b配合，实现自动取0
                     */
                for (int v = 0; v < numVertices; v++) {
                    GRBLinExpr constraint3c = new GRBLinExpr();

                    constraint3c.addTerm(1,y[v]);

                    for (int u = 0; u < numVertices; u++) {
                        if (adjMatrix.get(v).get(u) == 1) {
                            constraint3c.addTerm(1.0, x[u][0]);
                        }
                    }

                    constraint3c.addTerm(-1,x[v][1]);

                    model.addConstr(constraint3c, GRB.GREATER_EQUAL, 0,"约束3c：满足极大性的全部标记y_v=1");

                }

            // 约束4:至少一个点满足极大性
            // sum(y_v) >= 1

            GRBLinExpr constraint4 = new GRBLinExpr();
            for (int i = 0; i < numVertices; i++) {
                constraint4.addTerm(1.0, y[i]);
            }
            model.addConstr(constraint4, GRB.GREATER_EQUAL, 1,"约束4:至少一个点满足极大性");

            // 目标函数：最小化赋值总和
            GRBLinExpr objective = new GRBLinExpr();
            for (int v = 0; v < numVertices; v++) {
                objective.addTerm(0.0, x[v][0]);
                objective.addTerm(1.0, x[v][1]);
                objective.addTerm(2.0, x[v][2]);
            }
            model.setObjective(objective, GRB.MINIMIZE);

            // 优化模型
            model.optimize();

            // 输出结果
            for (int v = 0; v < numVertices; v++) {
                System.out.print("Vertex " + v + ": x_0 = " + (int)x[v][0].get(GRB.DoubleAttr.X));
                System.out.print(", x_1 = " + (int)x[v][1].get(GRB.DoubleAttr.X));
                System.out.print(", x_2 = " + (int)x[v][2].get(GRB.DoubleAttr.X)+")");
                System.out.println(", y="  + (int)y[v].get(GRB.DoubleAttr.X));
            }

            System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));
            System.out.println("Runtime: " + model.get(GRB.DoubleAttr.Runtime));


            //画图
            // 创建 GraphStream 的图
            org.graphstream.graph.Graph gsGraph = new SingleGraph("Undirected Graph");
            // 设置布局算法和样式
            gsGraph.addAttribute("ui.stylesheet", "node { fill-color: grey; size: 15px; text-size: 10px; text-color: black; } edge { fill-color: grey; }");

            // 启用高质量显示
            gsGraph.addAttribute("ui.quality");
            gsGraph.addAttribute("ui.antialias");


            int v= graph.getV();
            // 添加顶点
            for (int i = 0; i < v; i++) {
                org.graphstream.graph.Node node = gsGraph.addNode(String.valueOf(i));
                String label;
                if((int)y[i].get(GRB.DoubleAttr.X)==1){label = ", T";}
//                else{label = "F";}
                else{label = "";}
                // 为每个节点添加编号作为标签，加上gurobi计算结果
                if ((int)x[i][1].get(GRB.DoubleAttr.X) == 1){
                    node.addAttribute("ui.label", "("+String.valueOf(i)+")"+ " 1 " +label);
                } else if ((int)x[i][2].get(GRB.DoubleAttr.X) == 1) {
                    node.addAttribute("ui.label", "("+String.valueOf(i)+")"+ " 2 " +label);
                }else{
                    node.addAttribute("ui.label", "("+String.valueOf(i)+")"+ " 0 " +label);
                }
            }
            // 添加边
            for (Pair<Integer, Integer> edge : graph.getEdges()) {
                Integer source = edge.getFirst();
                Integer target = edge.getSecond();
                String edgeId = source + "-" + target;

                // 防止重复边
                if (gsGraph.getEdge(edgeId) == null) {
                    gsGraph.addEdge(edgeId, source.toString(), target.toString());
                }
            }
            // 显示图形并设置窗口标题
//           gsGraph.display();
            Viewer viewer = gsGraph.display();
            viewer.setCloseFramePolicy(Viewer.CloseFramePolicy.EXIT); // 设置窗口关闭策略

            // 添加注释（例如可以添加作为图的一部分显示）
            org.graphstream.graph.Node commentNode = gsGraph.addNode("comment");
            commentNode.addAttribute("ui.label", "(i),1,T 分别为顶点编号，赋值，该点闭邻域无0");
            commentNode.addAttribute("ui.style", "text-alignment: at-right; text-color: black; fill-color: rgba(255, 255, 255, 0);");
            commentNode.setAttribute("xyz", 0, v / 2.0, 0);  // 将注释节点放置在合适的地方


            // 清理
            model.dispose();
            env.dispose();
        } catch (GRBException e) {
            e.printStackTrace();
        }
    }

    @LogExecutionTime
    public static void ILP_MDRD_Apprximate(Graph graph){
        //输出文件的路径
        String resultFileName = "C:\\Users\\Administrator\\Desktop\\MDRD_Approximate_Result.txt";
        try {
            // 创建环境
            GRBEnv env = new GRBEnv(true);
            env.set(GRB.IntParam.OutputFlag, 0); // 设置不输出信息到控制台
            env.set("logFile", "src/main/java/gzhu/yh/logger/ILP_MDRD.log"); //设置日志文件
            env.start();

            // 创建模型
            GRBModel model = new GRBModel(env);
            model.set(GRB.StringAttr.ModelName, "ILP_MDRD");
            // 获取图的属性
            int numVertices = graph.getV(); // 顶点数
            List<List<Integer>> adjMatrix = graph.getAdjMatrix(); // 邻接矩阵

            // 定义变量
            GRBVar[][] x = new GRBVar[numVertices][3]; // 0: x_v^0, 1: x_v^1, 2: x_v^2
            GRBVar[] y = new GRBVar[numVertices]; // 辅助变量 y_w, 当前点是否满足极大性约束

            for (int v = 0; v < numVertices; v++) {
                x[v][0] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_0");//赋值为0
                x[v][1] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_1");//赋值为1
                x[v][2] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_2");//赋值为2
                y[v] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "y_" + v);//满足极大性
            }

            // 约束1：唯一赋值
            // x0_v + x1_v + x2_v=1
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr constraint1 = new GRBLinExpr();
                constraint1.addTerm(1.0, x[v][0]);
                constraint1.addTerm(1.0, x[v][1]);
                constraint1.addTerm(1.0, x[v][2]);
                model.addConstr(constraint1, GRB.EQUAL, 1.0,"约束1：唯一赋值");
            }

            // 约束2：罗马约束
            // x0_v <= \sum_{u\in N(v) x2_u}
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr constraint2 = new GRBLinExpr();
                for (int u = 0; u < numVertices; u++) {
                    if (adjMatrix.get(v).get(u) == 1) {
                        constraint2.addTerm(1.0, x[u][2]);
                    }
                }
                model.addConstr(x[v][0], GRB.LESS_EQUAL, constraint2,"约束2:罗马约束");
            }
            //约束合集：极大性约束描述
                // 约束3a：极大性出自V_1
                // y_v <= x1_v  for v in V
                for (int v = 0; v < numVertices; v++) {
                    GRBLinExpr constraint3a = new GRBLinExpr();
                    constraint3a.addTerm(1.0,x[v][1]);
                    model.addConstr(y[v], GRB.LESS_EQUAL, constraint3a,"约束3a：极大性出自V_1");
                }

                // 约束3b：满足极大性则邻点无0
                // sum_{u\in N(v)}(x0_u) <= (1-y_v)deg(v) for v in V
                /*
                    y_v=1时，右侧为0，强制所有邻点x0_u = 0
                    y_v=0时，约束相当于没有
                 */
                for (int v = 0; v < numVertices; v++) {
                    GRBLinExpr constraint3b = new GRBLinExpr();

                    int deg_v=0;//v的度
                    for (int u = 0; u < numVertices; u++) {
                        if (adjMatrix.get(v).get(u) == 1) {
                            constraint3b.addTerm(1.0, x[u][0]);
                            deg_v++;
                        }
                    }
                    constraint3b.addTerm(deg_v,y[v]);
                    model.addConstr(constraint3b, GRB.LESS_EQUAL, deg_v,"约束3b：满足极大性则邻点无0");

                }
                // 约束3c：满足极大性的全部标记y_v=1
                // y_v >= x1_v - sum_{u\in N(v)}(x0_u) for v in V
                    /*
                        当且仅当 x1_v=1且所有邻点x_0u = 0才有y_v=1
                        其他情况需与约束3a、3b配合，实现自动取0
                     */
                for (int v = 0; v < numVertices; v++) {
                    GRBLinExpr constraint3c = new GRBLinExpr();

                    constraint3c.addTerm(1,y[v]);

                    for (int u = 0; u < numVertices; u++) {
                        if (adjMatrix.get(v).get(u) == 1) {
                            constraint3c.addTerm(1.0, x[u][0]);
                        }
                    }

                    constraint3c.addTerm(-1,x[v][1]);

                    model.addConstr(constraint3c, GRB.GREATER_EQUAL, 0,"约束3c：满足极大性的全部标记y_v=1");

                }

            // 约束4:至少一个点满足极大性
            // sum(y_v) >= 1

            GRBLinExpr constraint4 = new GRBLinExpr();
            for (int i = 0; i < numVertices; i++) {
                constraint4.addTerm(1.0, y[i]);
            }
            model.addConstr(constraint4, GRB.GREATER_EQUAL, 1,"约束4:至少一个点满足极大性");

            // 目标函数：最小化赋值总和
            GRBLinExpr objective = new GRBLinExpr();
            for (int v = 0; v < numVertices; v++) {
                objective.addTerm(0.0, x[v][0]);
                objective.addTerm(1.0, x[v][1]);
                objective.addTerm(2.0, x[v][2]);
            }
            model.setObjective(objective, GRB.MINIMIZE);

            // 优化模型
            model.optimize();

//            // 输出结果
//            for (int v = 0; v < numVertices; v++) {
//                System.out.print("Vertex " + v + ": x_0 = " + (int)x[v][0].get(GRB.DoubleAttr.X));
//                System.out.print(", x_1 = " + (int)x[v][1].get(GRB.DoubleAttr.X));
//                System.out.print(", x_2 = " + (int)x[v][2].get(GRB.DoubleAttr.X)+")");
//                System.out.println(", y="  + (int)y[v].get(GRB.DoubleAttr.X));
//            }

//            System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));
//            System.out.println("Runtime: " + model.get(GRB.DoubleAttr.Runtime));
            /*输出结果*/
            //ILP计算结果
            int accurateCount=0;
            for (int v = 0; v < numVertices; v++) {
                accurateCount+= (int)x[v][1].get(GRB.DoubleAttr.X) + (int)x[v][2].get(GRB.DoubleAttr.X)*2;
            }
            System.out.print("gurobi 计算结果是 " + accurateCount+ "\t");


            //近似算法计算结果
            //调用近似算法
            GreedyMRDF greedyMRDF = SpringContextUtil.getBean(GreedyMRDF.class);
            int[] f = greedyMRDF.greedySolve(graph);
            int apprCost = Arrays.stream(f).sum();
            System.out.print("近似算法顶点赋值总成本: " + apprCost + "\t");

            //近似比结果
            double raito = apprCost/(double)accurateCount;
            System.out.println("近似比="+ raito);

//            Integer DELTA =0;
//            for (List<Integer> list : graph.getAdjList()) {
//                if(list.size()>=DELTA){
//                    DELTA= list.size();
//                }
//            }
//            System.out.println("理论近似比="+ Math.log(DELTA));


            // 创建文件对象
            File file = new File(resultFileName);

            BufferedWriter writer = new BufferedWriter(new FileWriter(file,true)); // true 表示追加模式

            if (file.exists()) {
                writer.newLine(); // 文件已存在时，换行再追加内容
            }

            writer.write("图的类型为：" + graph.getGraphType() + "; 顶点数为 " + graph.getV()  + "\t ");
            writer.newLine();  // 换行

            writer.write("gurobi 计算结果是 " + accurateCount + "\t " +
                    "近似算法顶点赋值总成本: " + apprCost + "\t " +
//                    "理论近似比=" + Math.log(DELTA)+ "\t "+
                    "实际近似比="+ raito +"\t"
            );

//            if (raito > Math.log(DELTA)){
//                writer.write("False");
//            }



            // 清理
            model.dispose();
            env.dispose();
            writer.flush();
            writer.close();
        } catch (GRBException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
            System.err.println("写入文件时出错: " + e.getMessage());
        }
    }
}
