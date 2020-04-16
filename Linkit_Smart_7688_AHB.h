enum AHB_assert_bus{
  AHB_R_BUS, // 0: Bus transaction is asserted by Rbus master interface, can access DRAM and peripheral registers
  AHB_P_BUS // 1: Bus transaction is asserted by Pbus master interface, can peripheral registers only.
};

enum AHB_size{
  AHB_WORD = 0b10
};

enum AHB_RW{
  AHB_READ,
  AHB_WRITE
};
