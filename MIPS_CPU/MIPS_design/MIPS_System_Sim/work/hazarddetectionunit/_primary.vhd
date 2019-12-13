library verilog;
use verilog.vl_types.all;
entity hazarddetectionunit is
    port(
        dc_rs           : in     vl_logic_vector(4 downto 0);
        dc_rt           : in     vl_logic_vector(4 downto 0);
        ex_rt           : in     vl_logic_vector(4 downto 0);
        ex_memtoreg     : in     vl_logic;
        pcwrite_en      : out    vl_logic;
        dc_en           : out    vl_logic;
        control_en      : out    vl_logic
    );
end hazarddetectionunit;
