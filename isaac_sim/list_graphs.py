from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.graph.core as og

omni.usd.get_context().open_stage("/home/lighthouse/ros2_ws/src/lighthouse/lighthouse_isaac_hospital_scene/isaac_hospital_scene_spot.usd")
simulation_app.update()
simulation_app.update()

stage = omni.usd.get_context().get_stage()

for prim in stage.Traverse():
    if prim.GetTypeName() == "OmniGraph":
        print(f"\n=== Graph: {prim.GetPath()} ===")
        graph = og.get_graph_by_path(str(prim.GetPath()))
        if graph:
            for node in graph.get_nodes():
                print(f"  Node: {node.get_prim_path()}  type: {node.get_type_name()}")
                for attr in node.get_attributes():
                    # Print connections
                    if attr.get_upstream_connection_count() > 0:
                        for src in attr.get_upstream_connections():
                            src_path = f"{src.get_node().get_prim_path()}.{src.get_name()}"
                            dst_path = f"{attr.get_node().get_prim_path()}.{attr.get_name()}"
                            print(f"    CONNECT {src_path} -> {dst_path}")
                    # Safely read value
                    try:
                        val = og.Controller.get(attr)
                        if val is not None and str(val) not in ["None", "", "0", "False"]:
                            print(f"    SET {attr.get_name()} = {val}")
                    except Exception:
                        pass

simulation_app.close()
