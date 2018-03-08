function [result]= parallel_plotting(arg1)

  %% ===========================================================================
  %% DECLARATION/INITIALIZATION/PREALOCATION OF VARIABLES
  %% ===========================================================================

  old_endresult = zeros(150000,1);
  counter_watchdog = 0 ;


  %% ===========================================================================
  %% REAL TIME PLOT SETUP  (Soon to be phased out due to multithreading)
  %% ===========================================================================

  figure(1);
  plot(endresult)
  title('Sinal')
  xlabel('Samples')
  ylabel('Bit')
  axis manual
  %set(grafico, 'XData', 0:150000, 'YData', -1:2)
  axis([0 (length(endresult)) -0.1 1.1])
  hold off;


  %% ===========================================================================
  %% MAIN CYCLE
  %% ===========================================================================



  while counter_watchdog < 5;

    if endresult == old_endresult
      counter_watchdog = counter_watchdog + 1;
    else
      plot(endresult)
      old_endresult = endresult;
      counter_watchdog = 0;

  end

  result = labindex;

end
