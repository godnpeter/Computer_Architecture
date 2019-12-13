library verilog;
use verilog.vl_types.all;
entity forwardingunit is
    port(
        ex_rs           : in     vl_logic_vector(4 downto 0);
        ex_rt           : in     vl_logic_vector(4 downto 0);
        dc_rs           : in     vl_logic_vector(4 downto 0);
        dc_rt           : in     vl_logic_vector(4 downto 0);
        mem_rd          : in     vl_logic_vector(4 downto 0);
        wb_rd           : in     vl_logic_vector(4 downto 0);
        mem_regwrite    : in     vl_logic;
        wb_regwrite     : in     vl_logic;
        regdst          : in     vl_logic;
        memtoreg        : in     vl_logic;
        pcsrc           : in     vl_logic;
        dc_srca2        : out    vl_logic;
        dc_writedata    : out    vl_logic;
        ex_srca         : out    vl_logic_vector(1 downto 0);
        ex_srcb         : out    vl_logic_vector(1 downto 0)
    );
end forwardingunit;
