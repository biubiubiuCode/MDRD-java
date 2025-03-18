package gzhu.yh.model.Roman12Domination;

import com.gurobi.gurobi.GRB;
import com.gurobi.gurobi.GRBEnv;
import com.gurobi.gurobi.GRBException;
import com.gurobi.gurobi.GRBLinExpr;
import com.gurobi.gurobi.GRBModel;
import com.gurobi.gurobi.GRBVar;
import gzhu.yh.graphsModel.Graph;

import java.util.List;

/**
 * @author wendao
 * @since 2024-09-11
 * maxiaml roman domiation的ILP方程
 *      varialble:
 *
 *      minumum:
 *      subject to:
 *
 **/
public class ILP_RD12_Modify {
    public static void ILP_RD12(Graph graph){
        try {
            // 创建环境
            GRBEnv env = new GRBEnv(true);
            env.set("logFile", "src/main/java/gzhu/yh/logger/ILP_MDRD.log"); //设置日志文件
            env.start();

            // 创建模型
            GRBModel model = new GRBModel(env);
            model.set(GRB.StringAttr.ModelName, "ILP_RD12");
            // 获取图的属性
            int numVertices = graph.getV(); // 顶点数
            List<List<Integer>> adjMatrix = graph.getAdjMatrix(); // 邻接矩阵

            // 定义变量
            GRBVar[][] x = new GRBVar[numVertices][2]; // 0: x_v^0, 1: x_v^1, 2: x_v^2

            for (int v = 0; v < numVertices; v++) {
                x[v][0] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_0");
                x[v][1] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_1");
                x[v][2] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_2");
            }

            // 约束1：每个顶点只能被赋值为0、1或2其中的一个
            // x0_v + x1_v + x2_v=0
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr constraint1 = new GRBLinExpr();
                constraint1.addTerm(1.0, x[v][0]);
                constraint1.addTerm(1.0, x[v][1]);
                constraint1.addTerm(1.0, x[v][2]);
                model.addConstr(constraint1, GRB.EQUAL, 1.0,"约束1：每个顶点只能被赋值为0、1或2其中的一个");
            }

            // 约束2：赋值为0的顶点至少有一个邻接点赋值为2
            // x0_v <= sum(x2_u) for u in N(v)
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr constraint2 = new GRBLinExpr();
                for (int u = 0; u < numVertices; u++) {
                    if (adjMatrix.get(v).get(u) == 1) {
                        constraint2.addTerm(1.0, x[u][2]);
                    }
                }
                model.addConstr(x[v][0], GRB.LESS_EQUAL, constraint2,"约束2：赋值为0的顶点至少有一个邻接点赋值为2");
            }

            //RD12的ILP需要一个充分大的整数
            int M = 100000;//TODO 理论上为Integer.MAX_VALUE.鉴于点数没超过1000，M值取100 000
            // 约束3：赋值为0的顶点至多有2个邻接点赋值为2
            // x0_v <= sum(x2_u) for u in N(v)  sum(x2_u) <= 2 + M(1-x0_v) for u in N(v)  M为一个充分大的整数
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr constraint2 = new GRBLinExpr();
                for (int u = 0; u < numVertices; u++) {
                    if (adjMatrix.get(v).get(u) == 1) {
                        constraint2.addTerm(1.0, x[u][2]);
                    }
                }
//                GRBLinExpr constraint3 = new GRBLinExpr();
//                constraint3.addTerm(-1.0 * M, x[v][0]);
                constraint2.addTerm(M,x[v][0]);
//                model.addConstr(constraint2 - constraint3 , GRB.LESS_EQUAL,2 + M ,"约束2：赋值为0的顶点至少有一个邻接点赋值为2");
                model.addConstr(constraint2 , GRB.LESS_EQUAL,2 + M ,"约束3：赋值为0的顶点至多有2个邻接点赋值为2");
            }





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
            }

            // 清理
            model.dispose();
            env.dispose();

        } catch (GRBException e) {
            e.printStackTrace();
        }
    }
}
