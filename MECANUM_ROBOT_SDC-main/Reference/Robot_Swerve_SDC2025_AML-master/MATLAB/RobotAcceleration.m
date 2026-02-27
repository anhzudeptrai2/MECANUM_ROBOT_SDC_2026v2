classdef RobotAcceleration
    properties
        currentVelocity
        targetVelocity
        previousTarget
        duration
        tau
        timeElapsed
    end

    methods
        function obj = RobotAcceleration(initialTarget, duration)
            obj.currentVelocity = 0.0;
            obj.targetVelocity = initialTarget;
            obj.previousTarget = initialTarget;
            obj.duration = duration;
            obj.tau = duration / 5.0;
            obj.timeElapsed = 0.0;
        end

        function obj = setTarget(obj, newTargetVelocity, newDuration)
            obj.previousTarget = obj.currentVelocity;
            obj.targetVelocity = newTargetVelocity;
            obj.duration = newDuration;
            obj.tau = newDuration / 5.0;
            obj.timeElapsed = 0.0;
        end

        function [obj, velocity, acceleration] = update(obj, dt)
            obj.timeElapsed = obj.timeElapsed + dt;

            if obj.timeElapsed > obj.duration
                obj.timeElapsed = obj.duration;
            end

            velocityRange = obj.targetVelocity - obj.previousTarget;
            velocity = obj.previousTarget + velocityRange * (1 - exp(-obj.timeElapsed / obj.tau));
            acceleration = (velocityRange / obj.tau) * exp(-obj.timeElapsed / obj.tau);

            obj.currentVelocity = velocity; % Cập nhật vận tốc hiện tại
        end


    end

    methods (Static)
        function plotVelocityAndAcceleration(initialTarget, duration, dt)
            robot = RobotAcceleration(initialTarget, duration);
            time = 0:dt:duration;
            velocity = zeros(size(time));
            acceleration = zeros(size(time));

            for i = 1:length(time)
                [robot, velocity(i), acceleration(i)] = robot.update(dt);
            end

            figure;
            subplot(2,1,1);
            plot(time, velocity, 'b', 'LineWidth', 2);
            xlabel('Thời gian (s)');
            ylabel('Vận tốc (m/s)');
            title('Biểu đồ Vận tốc theo thời gian');
            grid on;

            subplot(2,1,2);
            plot(time, acceleration, 'r', 'LineWidth', 2);
            xlabel('Thời gian (s)');
            ylabel('Gia tốc (m/s^2)');
            title('Biểu đồ Gia tốc theo thời gian');
            grid on;
        end
    end
end