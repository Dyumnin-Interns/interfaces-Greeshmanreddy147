import os
from pathlib import Path
import cocotb
from cocotb.runner import get_runner
from cocotb.triggers import ClockCycles, RisingEdge
from cocotb.clock import Clock
from cocotb.log import logging, SimLog
from cocotb_bus.drivers import BusDriver
from cocotb_coverage.coverage import CoverPoint, CoverCross, coverage_db
import random as rnd
import constraint

@CoverPoint('top.a' , xf = lambda x, y : x, bins = [0,1])
@CoverPoint('top.b' , xf = lambda x, y : y, bins = [0,1])
@CoverCross('top.cross.ab' , items = ['top.a','top.b'])
def ab_cover(a,b):
    pass

@CoverPoint('top.rd_add' , xf = lambda rd_add , rd_en , wd_add , wd_en ,wd_data : rd_add , bins = [0,1,2,3])
@CoverPoint('top.rd_en' , xf = lambda rd_add , rd_en , wd_add , wd_en ,wd_data : rd_en , bins = [0,1])
@CoverPoint('top.wd_add' , xf = lambda rd_add , rd_en , wd_add , wd_en ,wd_data : wd_add , bins = [4,5])
@CoverPoint('top.wd_en' , xf = lambda rd_add , rd_en , wd_add , wd_en ,wd_data : wd_en , bins = [0,1])
@CoverPoint('top.wd_data' , xf = lambda rd_add , rd_en , wd_add , wd_en ,wd_data : wd_data , bins = [0,1])
@CoverCross('top.cross.w' , items = ['top.wd_add','top.wd_data','top.wd_en'])
@CoverCross('top.cross.r',items = ['top.rd_add','top.rd_en'])
def w_r_cross(wd_data,wd_en,wd_add,rd_add,rd_en):
    pass

class write_driver(BusDriver):
    _signals = ['CLK','RST_N','write_address','write_data','write_en','write_rdy']
    def __init__(self , name, entity ):
        self.name = name
        self.entity = entity
        self.CLK = self.entity.CLK

    async def _driver_send(self,transaction):
        await RisingEdge(self.CLK)
        if self.entity.write_rdy.value !=1 :
            await RisingEdge(self.entity.write_rdy)
        self.entity.write_en.value = 1
        self.entity.write_address.value = transaction.get('add')
        self.entity.write_data.value = transaction.get('val')
        await RisingEdge(self.CLK)
        self.entity.write_en.value = 0

class read_driver(BusDriver):
    _signals = ['CLK','RST_N','read_address','read_en','read_rdy','read_data']
    def __init__(self,name,entity):
        self.name = name
        self.entity = entity
        self.CLK = self.entity.CLK
    async def _driver_send(self,transaction):
        await RisingEdge(self.CLK)
        if self.entity.read_rdy.value !=1:
            await RisingEdge(self.entity.read_rdy)
        self.entity.read_en.value = 1
        self.entity.read_address.value = transaction.get('add')
        await RisingEdge(self.CLK)
        self.entity.read_en.value = 0

class tb:
    def __init__(self,name,entity,log):
        self.log = log
        self.name = name
        self.entity = entity
        self.CLK = self.entity.CLK
        self.stats = []
        self.ref_add = { 'a_status': 0 ,'b_status' : 1 , 'y_status' : 2, 'y_output' : 3, 'a_data' : 4, 'b_data' : 5}
        self.writer = write_driver('write_FIFO',entity)
        self.reader = read_driver('read_FIFO', entity)
        
    async def reset(self):
        await RisingEdge(self.CLK)
        self.entity.write_en.value = 0
        self.entity.write_address.value = 0
        self.entity.write_data.value = 0
        self.entity.read_en.value = 0
        self.entity.read_address.value = 0
        self.entity.read_data.value = 0
        self.entity.RST_N.value = 1
        await ClockCycles(self.CLK,3)
        self.entity.RST_N.value = 0
        await ClockCycles(self.CLK,3)
        self.entity.RST_N.value = 1
        await RisingEdge(self.CLK)

    def stat(self,add,val):
        if add == 0:
            self.stats.append({'name': "as" ,'val' : ( "full" if val == 0 else 'empty')})
        if add == 1:
            self.stats.append({'name': "bs" , "val" : ( "full" if val == 0 else 'empty')})
        if add == 2:
            self.stats.append({'name': "ys" , "val" : ( "full" if val == 1 else 'empty')})
        if add == 3:
            self.stats.append({'name': "yr" , "val" : val})
        if add == 4:
            self.stats.append({'name': "aw" , "val" : val})
        if add == 5:
            self.stats.append({'name': "bw" , "val" : val})
    
    def con(self):
        self.s = constraint.Problem()
        self.s.addVariable("write_en", [0,1])
        self.s.addVariable("write_address", [4,5])
        self.s.addVariable("write_data", [0,1])
        self.s.addVariable("write_rdy", [1])
        self.s.addVariable("read_en", [0,1])
        self.s.addVariable("read_rdy", [1])
        self.s.addVariable("read_address", [3,2,1,0])
        self.s.addConstraint(
            lambda rd_en, rd_rdy, wd_en: rd_en == 1 if (rd_rdy == 1 and wd_en == 0) else rd_en == 0,
            ['read_en', 'read_rdy', 'write_en']
        )
        self.s.addConstraint(
            lambda wd_en, wd_rdy, rd_en: wd_en == 1 if (wd_rdy == 1 and rd_en == 0) else wd_en == 0,
            ['write_en', 'write_rdy', 'read_en']
        )
        self.p.addConstraint(
            lambda rd_addr, rd_en: True if rd_en == 1 else rd_addr in [0, 1],
            ['read_address', 'read_en']
        )

    
    def solve(self):
        self.con_obj = self.con()
        self.sols = self.s.getSolutions()
    
    def get_sols(self):
        return rnd.choice(self.sols) if self.sols else None

@cocotb.test()
async def dut_test(dut):
    cocotb.start_soon(Clock(dut.CLK,2,"ns").start())
    log = SimLog('interface_test')
    logging.getLogger().setLevel(logging.INFO)
    tbh = tb(name = "tb inst",entity = dut, log=log)
    await tbh.reset()

    await tbh.writer._driver_send(transaction={'add':4 ,'val':0})
    await tbh.writer._driver_send(transaction={'add':5 ,'val':0})
    ab_cover(0,0)
    await tbh.reader._driver_send(transaction = {'add':3,'val':0})
    
    await tbh.writer._driver_send(transaction={'add':4 ,'val':1})
    await tbh.writer._driver_send(transaction={'add':5 ,'val':0})
    ab_cover(1,0)
    await tbh.reader._driver_send(transaction = {'add':3,'val':0})
    
    await tbh.writer._driver_send(transaction={'add':4 ,'val':0})
    await tbh.writer._driver_send(transaction={'add':5 ,'val':1})
    ab_cover(0,1)
    await tbh.reader._driver_send(transaction = {'add':3,'val':0})

    await tbh.writer._driver_send(transaction={'add':4 ,'val':1})
    await tbh.writer._driver_send(transaction={'add':5 ,'val':1})
    ab_cover(1,1)
    await tbh.reader._driver_send(transaction = {'add':3,'val':0})
    log.debug(f"[functional] a:1 b:1 y:{dut.read_data.value}")

    tbh.solve()
    for i in range(100):
        x = tbh.get_sols()
        w_r_cross(x.get('write_data'),x.get("write_en"),x.get("write_address"),x.get('read_address'),x.get("read_en"))
        if x.get('read_en') == 1:
            await tbh.reader._driver_send(transaction = {'add':x.get('read_address') , 'val': 0})
            log.debug(f"[{i}][read operation] address : {x.get('read_address')} got data : {dut.read_data.value.integer}")
            tbh.stat(x.get('read_address'),dut.read_data.value.integer)
        elif x.get('write_en') == 1:
            await tbh.writer._driver_send(transaction = {'add':x.get('write_address') , 'val': x.get('write_data')})
            log.debug(f"[{i}][write operation] address : {x.get('write_address')} put data : x.get('write_data')")
            tbh.stat(x.get('write_address'),x.get('write_data'))
        await RisingEdge(dut.CLK)
    for i in tbh.stats:
        log.debug(f"{i}")
    
    coverage_db.report_coverage(log.info,bins=True)
    log.info(f"Functional Coverage: {coverage_db['top.cross.ab'].cover_percentage:.2f} %")
    log.info(f"Write Coverage: {coverage_db['top.cross.w'].cover_percentage:.2f} %")
    log.info(f"Read Coverage: {coverage_db['top.cross.r'].cover_percentage:.2f} %")
     


def start_build():
    sim = os.getenv("SIM","verilator")
    dut_dir = Path(__file__).resolve().parent.parent
    dut_dir = f"{dut_dir}/hdl"
    hdl_toplevel="dut"
    verilog_sources = [f"{dut_dir}/{hdl_toplevel}.v", f"{dut_dir}/FIFO1.v",f"{dut_dir}/FIFO2.v"]
    build_args = ["--trace", "--trace-fst"]
    
    runner = get_runner(sim)

    runner.build(
        hdl_toplevel=hdl_toplevel,
        verilog_sources=verilog_sources,
        build_args=build_args,
        waves=True,
        always=True
    )

    runner.test(
    test_module="dut_test",
        hdl_toplevel=hdl_toplevel,
        waves=True
    )

if __name__ == "__main__":
    start_build()
    








