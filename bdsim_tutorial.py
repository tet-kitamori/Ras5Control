"""
bdsim (Block Diagram Simulator) - 制御系シミュレーション実用例
"""

from bdsim import BDSim
import matplotlib
matplotlib.use('Agg')  # グラフィックス表示を無効化
import matplotlib.pyplot as plt

print("\n" + "=" * 70)
print("bdsim - 制御系シミュレーション実用例")
print("=" * 70)

# ========== 例1: 最小限 ==========
print("\n【例1】ステップ入力")
try:
    sim = BDSim()
    bd = sim.blockdiagram()
    
    step = bd.STEP(T=0, pos=1)
    scope = bd.SCOPE(title="Example1: Step")
    
    bd.connect(step, scope)
    bd.compile()
    
    print("  実行中...", end=" ")
    res = sim.run(bd, T=5, dt=0.1)
    print("✓ 完了")
except Exception as e:
    print(f"✗ {e}")


# ========== 例2: ゲイン ==========
print("\n【例2】ゲイン適用（×2.5）")
try:
    sim = BDSim()
    bd = sim.blockdiagram()
    
    step = bd.STEP(T=0, pos=1)
    gain = bd.GAIN(2.5)
    scope = bd.SCOPE(title="Example2: Gain")
    
    bd.connect(step, gain)
    bd.connect(gain, scope)
    bd.compile()
    
    print("  実行中...", end=" ")
    res = sim.run(bd, T=5, dt=0.1)
    print("✓ 完了")
except Exception as e:
    print(f"✗ {e}")


# ========== 例3: 加減算 ==========
print("\n【例3】複数入力の足し算")
try:
    sim = BDSim()
    bd = sim.blockdiagram()
    
    step = bd.STEP(T=0, pos=2)
    ramp = bd.RAMP()
    summer = bd.SUM('++')
    scope = bd.SCOPE(title="Example3: STEP+RAMP")
    
    bd.connect(step, summer[0])
    bd.connect(ramp, summer[1])
    bd.connect(summer, scope)
    bd.compile()
    
    print("  実行中...", end=" ")
    res = sim.run(bd, T=5, dt=0.1)
    print("✓ 完了")
except Exception as e:
    print(f"✗ {e}")


# ========== 例4: 積分器 ==========
print("\n【例4】積分器：定数を積分して → 直線")
try:
    sim = BDSim()
    bd = sim.blockdiagram()
    
    const = bd.CONSTANT(2.0)
    integrator = bd.INTEGRATOR()
    scope = bd.SCOPE(title="Example4: Integrator")
    
    bd.connect(const, integrator)
    bd.connect(integrator, scope)
    bd.compile()
    
    print("  実行中...", end=" ")
    res = sim.run(bd, T=5, dt=0.01)
    print("✓ 完了")
except Exception as e:
    print(f"✗ {e}")


# ========== 例5: 伝達関数 ==========
print("\n【例5】伝達関数：1次遅れ G(s)=1/(s+1)")
try:
    sim = BDSim()
    bd = sim.blockdiagram()
    
    step = bd.STEP(T=0, pos=1)
    plant = bd.LTI_SISO([1], [1, 1])
    scope = bd.SCOPE(title="Example5: 1st-Order")
    
    bd.connect(step, plant)
    bd.connect(plant, scope)
    bd.compile()
    
    print("  実行中...", end=" ")
    res = sim.run(bd, T=10, dt=0.01)
    print("✓ 完了")
except Exception as e:
    print(f"✗ {e}")


# ========== 例6: P制御 ==========
print("\n【例6】フィードバック制御：P制御 (Kp=3)")
try:
    sim = BDSim()
    bd = sim.blockdiagram()
    
    ref = bd.STEP(T=0, pos=1)
    error = bd.SUM('+-')
    controller = bd.GAIN(3)
    plant = bd.LTI_SISO([1], [1, 1])
    scope = bd.SCOPE(title="Example6: P-Control")
    
    bd.connect(ref, error[0])
    bd.connect(plant, error[1])
    bd.connect(error, controller)
    bd.connect(controller, plant)
    bd.connect(plant, scope)
    bd.compile()
    
    print("  実行中...", end=" ")
    res = sim.run(bd, T=10, dt=0.01)
    print("✓ 完了")
except Exception as e:
    print(f"✗ {e}")


# ========== 例7: PID制御 ==========
print("\n【例7】PID制御 (Kp=2, Ki=0.3, Kd=0.2)")
try:
    sim = BDSim()
    bd = sim.blockdiagram()
    
    # PIDパラメータ
    Kp, Ki, Kd = 2.0, 0.3, 0.2
    
    ref = bd.STEP(T=0, pos=1)
    error = bd.SUM('+-')
    
    # P項
    gain_p = bd.GAIN(Kp)
    
    # I項
    integrator = bd.INTEGRATOR()
    gain_i = bd.GAIN(Ki)
    
    # D項
    deriv = bd.DERIV(alpha=1)
    gain_d = bd.GAIN(Kd)
    
    # 合算
    pid_sum = bd.SUM('+++')
    
    # プラント
    plant = bd.LTI_SISO([1], [1, 1])
    scope = bd.SCOPE(title="Example7: PID-Control")
    
    # 接続
    bd.connect(ref, error[0])
    bd.connect(error, gain_p)
    bd.connect(error, integrator)
    bd.connect(error, deriv)
    
    bd.connect(gain_p, pid_sum[0])
    bd.connect(integrator, gain_i)
    bd.connect(gain_i, pid_sum[1])
    bd.connect(deriv, gain_d)
    bd.connect(gain_d, pid_sum[2])
    
    bd.connect(pid_sum, plant)
    bd.connect(plant, error[1])
    bd.connect(plant, scope)
    bd.compile()
    
    print("  実行中...", end=" ")
    res = sim.run(bd, T=10, dt=0.01)
    print("✓ 完了")
except Exception as e:
    print(f"✗ {e}")


# ========== 使用例の要点 ==========
print("\n" + "=" * 70)
print("bdsim の基本パターンと利用可能なブロック")
print("=" * 70)

print("""
【基本的な使い方】
  
  1. シミュレータとブロック図を作成
     sim = BDSim()
     bd = sim.blockdiagram()
  
  2. ブロックを生成
     step = bd.STEP(T=0, pos=1)
     gain = bd.GAIN(2.0)
  
  3. ブロックを接続
     bd.connect(step, gain)
  
  4. コンパイル（重要！）
     bd.compile()
  
  5. シミュレーション実行
     res = sim.run(bd, T=10, dt=0.01)

【入力ブロック】
  • STEP(T, pos)      - ステップ入力（時刻T で値pos に）
  • RAMP()            - ランプ入力（斜め上昇）
  • CONSTANT(value)   - 定数値
  • WAVEFORM(type)    - 波形信号

【動的ブロック】
  • INTEGRATOR()      - 積分器 (∫)
  • DERIV(alpha)      - 微分器 (d/dt, フィルタ定数α)
  • LTI_SISO(num,den) - 伝達関数 G(s) = num/den
    例: G(s) = 1/(s+1) → LTI_SISO([1], [1, 1])
  • LTI_SS(A,B,C,D)   - 状態空間表現

【静的ブロック】
  • GAIN(K)           - ゲイン（K倍）
  • SUM(spec)         - 加減算
    例: '++' → 入力1 + 入力2
        '+-' → 入力1 - 入力2
  • PROD()            - 乗算
  • CLIP(min, max)    - リミッター

【出力・監視】
  • SCOPE()           - グラフ表示
  • PRINT()           - コンソール出力

【複数入力への接続】
  
  summer = bd.SUM('++')  # 3入力足し算
  bd.connect(input1, summer[0])
  bd.connect(input2, summer[1])
  bd.connect(input3, summer[2])

【フィードバックループの実装】
  
  error = bd.SUM('+-')
  controller = bd.GAIN(3)
  plant = bd.LTI_SISO([1], [1, 1])
  
  bd.connect(reference, error[0])
  bd.connect(plant, error[1])          # フィードバック
  bd.connect(error, controller)
  bd.connect(controller, plant)
""")

print("\n" + "=" * 70)
print("すべてのシミュレーションが完了しました！")
print("=" * 70 + "\n")
