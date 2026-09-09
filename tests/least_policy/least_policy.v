module least_policy(
    input b,
    output y
);
  wire n0;

  // Two resizable cells on one violating path with very different delay
  // contributions, so the offender ranking is observable in the TSV order.
  BIGBUF_sp0_X1 big_cell(.A(b), .Y(n0));
  SMALLBUF_sp0_X1 small_cell(.A(n0), .Y(y));
endmodule
