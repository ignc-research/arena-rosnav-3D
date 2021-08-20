# How to include further world files

1. Dowload the worldfile + ggf. needed models 
2. The `.world` file should be saved in [simulator_setup/worlds](simulator_setup/worlds). Create a file of the following structure:

```
├── simulator_setup/
|    └── wolds/
│        └── {NAME OF YOUR WORLD}
│            └── models/ # if needed
|            └── worlds/
|                └── {NAME OF YOUR WORLD}.word
```    
3. Insert the following line into your model between the `<world></world>` tags to enable dynamic obstacle steering via pedsim_ros

```bash
<plugin name="ActorPosesPlugin" filename="libActorPosesPlugin.so"/>
```
4. Create a map-file