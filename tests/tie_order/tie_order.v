module tie_order(
    input a,
    output y0,
    output y1,
    output y2,
    output y3,
    output y4,
    output y5
);
  // Equal-score offenders whose endpoint order differs from their name order,
  // so a pick order that follows path or map order shows up in the TSV.
  BUF_sp0_X1 u3(.A(a), .Y(y0));
  BUF_sp0_X1 u0(.A(a), .Y(y1));
  BUF_sp0_X1 u5(.A(a), .Y(y2));
  BUF_sp0_X1 u1(.A(a), .Y(y3));
  BUF_sp0_X1 u4(.A(a), .Y(y4));
  BUF_sp0_X1 u2(.A(a), .Y(y5));
endmodule
