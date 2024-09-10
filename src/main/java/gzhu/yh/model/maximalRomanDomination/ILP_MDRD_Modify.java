package gzhu.yh.model.maximalRomanDomination;

import com.gurobi.gurobi.GRB;
import com.gurobi.gurobi.GRBEnv;
import com.gurobi.gurobi.GRBException;
import com.gurobi.gurobi.GRBExpr;
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
public class ILP_MDRD_Modify {
    public static void ILP_MDRD(Graph graph){
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
            //MDRD的ILP需要先求各个顶点的度
            int[] deg = new int[numVertices];
            for (int i = 0; i < numVertices; i++) {
                deg[i]=graph.getAdjList().get(i).size();
            }

            // 定义变量
            GRBVar[][] x = new GRBVar[numVertices][3]; // 0: x_v^0, 1: x_v^1, 2: x_v^2
            GRBVar[] y = new GRBVar[numVertices]; // 辅助变量 y_w,但前点是否被V0控制

            for (int v = 0; v < numVertices; v++) {
                x[v][0] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_0");
                x[v][1] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_1");
                x[v][2] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + v + "_2");
                y[v] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "y_" + v);
            }

            // 每个顶点只能有一个赋值
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr vars = new GRBLinExpr();
                vars.addTerm(1.0, x[v][0]);
                vars.addTerm(1.0, x[v][1]);
                vars.addTerm(1.0, x[v][2]);
                model.addConstr(vars, GRB.EQUAL, 1.0,"每个顶点只能有一个赋值");
            }

            // 赋值为0的顶点必须连接到至少一个赋值为2的顶点
            for (int v = 0; v < numVertices; v++) {
                GRBLinExpr sum = new GRBLinExpr();
                for (int u = 0; u < numVertices; u++) {
                    if (adjMatrix.get(v).get(u) == 1) {
                        sum.addTerm(1.0, x[u][2]);
                    }
                }
                model.addConstr(x[v][0], GRB.LESS_EQUAL, sum,"赋值为0的顶点必须连接到至少一个赋值为2的顶点");
            }

            // 不被V0控制的点的性质
            for (int w = 0; w < numVertices; w++) {
                GRBLinExpr sum = new GRBLinExpr();
                for (int u = 0; u < numVertices; u++) {
                    if (adjMatrix.get(w).get(u) == 1) {
                        sum.addTerm(1.0, x[u][0]);
                    }
                }
                GRBLinExpr left = new GRBLinExpr();
                left.addTerm(1.0, x[w][1]);
                left.addTerm(1.0, x[w][2]);
                model.addConstr(left, GRB.GREATER_EQUAL, y[w],"不被V0控制的点的性质1");

                GRBLinExpr right = new GRBLinExpr();
                right.addConstant(deg[w]);
                right.addTerm(-1*deg[w], y[w]);
//                model.addConstr(sum, GRB.LESS_EQUAL, (1 - y[w]) * deg[w]);
                model.addConstr(sum, GRB.LESS_EQUAL, right,"不被V0控制的点的性质2");
            }

            // 至少有一个不被V0控制的点
            GRBLinExpr sum_y = new GRBLinExpr();
            for (int w = 0; w < numVertices; w++) {
                sum_y.addTerm(1.0, y[w]);
            }
            model.addConstr(sum_y, GRB.GREATER_EQUAL, 1,"至少有一个不被0控制的点");

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
                System.out.print("Vertex " + v + ": x_" + v + "_0 = " + x[v][0].get(GRB.DoubleAttr.X));
                System.out.print(", x_" + v + "_1 = " + x[v][1].get(GRB.DoubleAttr.X));
                System.out.print(", x_" + v + "_2 = " + x[v][2].get(GRB.DoubleAttr.X)+")");
                System.out.print(", y_" + v + y[v].get(GRB.DoubleAttr.X));
            }

            // 清理
            model.dispose();
            env.dispose();

        } catch (GRBException e) {
            e.printStackTrace();
        }
    }
}
