module wns_policy(
    input a,
    input b,
    output fixed_y,
    output opt_y
);
  wire n0;
  wire n1;
  wire n2;
  wire n3;
  wire n4;
  wire n5;
  wire n6;
  wire n7;
  wire n8;

  FIXED_X1 fixed_path(.A(a), .Y(fixed_y));

  BUF_sp0_X1 opt_path_0(.A(b), .Y(n0));
  BUF_sp0_X1 opt_path_1(.A(n0), .Y(n1));
  BUF_sp0_X1 opt_path_2(.A(n1), .Y(n2));
  BUF_sp0_X1 opt_path_3(.A(n2), .Y(n3));
  BUF_sp0_X1 opt_path_4(.A(n3), .Y(n4));
  BUF_sp0_X1 opt_path_5(.A(n4), .Y(n5));
  BUF_sp0_X1 opt_path_6(.A(n5), .Y(n6));
  BUF_sp0_X1 opt_path_7(.A(n6), .Y(n7));
  BUF_sp0_X1 opt_path_8(.A(n7), .Y(n8));
  BUF_sp0_X1 opt_path_9(.A(n8), .Y(opt_y));
endmodule
