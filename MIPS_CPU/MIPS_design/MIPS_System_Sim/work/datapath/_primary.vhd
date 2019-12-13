library verilog;
use verilog.vl_types.all;
entity datapath is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        signext         : in     vl_logic;
        shiftl16        : in     vl_logic;
        memtoreg        : in     vl_logic;
        pcsrc           : in     vl_logic;
        alusrc          : in     vl_logic;
        regdst          : in     vl_logic;
        memwriteff      : in     vl_logic;
        regwrite        : in     vl_logic;
        jump            : in     vl_logic;
        jreg            : in     vl_logic;
        jlink           : in     vl_logic;
        alucontrol      : in     vl_logic_vector(2 downto 0);
        memwrite        : out    vl_logic;
        zero            : out    vl_logic;
        pc              : out    vl_logic_vector(31 downto 0);
        instr           : in     vl_logic_vector(31 downto 0);
        aluout          : out    vl_logic_vector(31 downto 0);
        writedata3      : out    vl_logic_vector(31 downto 0);
        dc_en           : out    vl_logic;
        control_en      : out    vl_logic;
        readdata        : in     vl_logic_vector(31 downto 0)
    );
end datapath;
