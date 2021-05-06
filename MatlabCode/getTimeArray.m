function [timearray] = getTimeArray(points,t_i, t_f)
distance = [];
timearray = [0];
totalT = t_f-t_i;
    for i = 1:length(points(1,:))-1
        distance(end+1) = norm(points(:,i+1)-points(:,i));
    end
    for i = 1:length(distance)
        timearray(i+1) = distance(i)/sum(distance) *totalT + timearray(i);
    end
end