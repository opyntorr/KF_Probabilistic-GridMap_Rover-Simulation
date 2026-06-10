from pxr import Usd
stage = Usd.Stage.Open("/home/opyntorr/isaacsim_assets/Assets/Isaac/4.5/Isaac/Environments/Modular_Warehouse/Props/warehouse_h10m_center.usd")
for prim in stage.GetPseudoRoot().GetChildren():
    print(prim.GetName())
    for child in prim.GetChildren():
        print("  -", child.GetName())
