classdef timer_queue
    %TIMER_QUEUE Maintains a FIFO Queue of integer timers that get appended to a log
    % This class creates the ability to have a queue of timers for agents,
    %   in a discrete time simulation. It generates a log of waiting times
    %   for all agents in order of arrival, and maintains past and current
    %   timer information in one storage object.
    
    properties
        TimeVector;
        CurrArrival;
        CurrDeparture;
    end
    
    methods
        function self = timer_queue()
            %TIMER Construct an instance of this class
            %   Detailed explanation goes here
            self.TimeVector = zeros(1, 0);
            self.CurrArrival = 1;
            self.CurrDeparture = 1;
        end
        
        function self = addTimers(self, amount)
            %ADDTIMERS add amount number of live timers to the queue
            self.TimeVector = [self.TimeVector, zeros(1, amount)];
            self.CurrArrival = self.CurrArrival + amount;
        end
        
        function self = dequeTimers(self)
            self.CurrDeparture = self.CurrArrival;
        end
        
        function self = wait(self, seconds)
            if (self.CurrDeparture == self.CurrArrival)
                return;
            end
            for i = self.CurrDeparture:(self.CurrArrival-1)
                self.TimeVector(i) = self.TimeVector(i) + seconds;
            end
        end
        
        function [awtMean, awtSd] = getAvgWaitTimeStats(self)
            n = self.CurrArrival;
            
            awtMean = mean(self.TimeVector);
            awtSd = std(self.TimeVector);
        end
        
        function [median, p90] = getAvgWaitPercentiles(self)
            x = sort(self.TimeVector);
            fifthPIndex = floor((90/100) * length(x));
            medianIndex = floor(length(x)/2);
            
            median = x(medianIndex);
            p90 = x(fifthPIndex);
        end
    end
end

