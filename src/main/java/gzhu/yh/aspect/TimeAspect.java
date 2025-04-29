package gzhu.yh.aspect;

import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;

@Aspect // 声明为切面
@Component // 注册为Spring Bean
public class TimeAspect {

    private static final Logger logger = LoggerFactory.getLogger(TimeAspect.class);

    // 环绕通知：匹配带有@LogExecutionTime注解的方法
    @Around("@annotation(gzhu.yh.annotation.LogExecutionTime)")
    public Object logExecutionTime(ProceedingJoinPoint joinPoint) throws Throwable {
        // 单位 毫秒 ms
//        long startTime = System.currentTimeMillis();
        //单位 纳秒 ns
        long startTime = System.nanoTime();
        try {
            // 执行目标方法
            return joinPoint.proceed();
        } finally {
            // 计算耗时（无论方法是否抛出异常）
//            long duration = System.currentTimeMillis() - startTime;
//            long duration = System.nanoTime() - startTime;
            long duration = (System.nanoTime() - startTime) / 1000;
            String methodName = joinPoint.getSignature().toShortString();
            System.out.print("方法"+methodName+" 执行耗时: "+duration +"  μs"+ "\t ");
            logger.info("方法 {} 执行耗时: {}  μs", methodName, duration);
        }
    }
}