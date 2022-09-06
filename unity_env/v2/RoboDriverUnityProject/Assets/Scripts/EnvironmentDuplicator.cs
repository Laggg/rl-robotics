using System;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MLAgentsDebugTool.Duplicator
{
    /// <summary>
    /// Duplicates the environment rightwards, taking into account bounds and safety distance.
    /// Duplicated environments can be placed on separate scenes with non interacting physics.
    /// NumEnvironments and MultiScene parameters are overriden with a hyperparameter provided in the python config file
    /// (through the academy environment parameters)
    /// </summary>
    public class EnvironmentDuplicator : MonoBehaviour
    {
        [SerializeField] 
        private GameObject environment;
       
        [SerializeField] [Tooltip("Overriden by python config")]
        private int defaultNumCopies;
        [SerializeField] [Tooltip("Overriden by python config")]
        private bool defaultMultiScene;

        private enum LayoutType
        {
            Grid,
            Horizontal,
            Vertical
        }
        
        [SerializeField]
        private LayoutType layoutType = LayoutType.Horizontal;
        
        [Header("One-dimensional")]
        [SerializeField] [Tooltip("Distance between bounds of spawned environments")]
        private float safetyDistance = 50;

        [Header("Grid")]
        [SerializeField] [Tooltip("How many env will be spawned horizontally")] 
        private int maxHorizontalGridSize = 10;
        
        [SerializeField] [Tooltip("Distance between bounds of spawned environments in the grid")]
        private Vector2 gridSafetyDistance = new Vector2(10, 10);

        private bool multiScene;
        private readonly List<PhysicsScene> spawnedPhysicsScenes = new List<PhysicsScene>(); 
        
        private void Start()
        {
            int numEnvironments = (int)Academy.Instance.EnvironmentParameters.GetWithDefault("environments_per_unity_process", defaultNumCopies);
            int multiSceneValue = (int)Academy.Instance.EnvironmentParameters.GetWithDefault("multi_scene", -1f);
            multiScene = multiSceneValue == -1 ? defaultMultiScene : multiSceneValue == 1;
        
            CreateSceneParameters csp = new CreateSceneParameters(LocalPhysicsMode.Physics3D);
            Bounds environmentBounds = CalculateEnvironmentBounds(environment);
            Vector3 firstEnvironmentPosition = environment.transform.position;
        
            for (int i = 0; i < numEnvironments; i++)
            {
                Vector3 pos = firstEnvironmentPosition + GetLayoutPosition(environmentBounds, i + 1);
                GameObject env = Instantiate(environment, pos, environment.transform.rotation);

                if (multiScene)
                {
                    Scene scene = SceneManager.CreateScene($"SpawnedEnv-{i}", csp);
                    SceneManager.MoveGameObjectToScene(env.gameObject, scene);
                    spawnedPhysicsScenes.Add(scene.GetPhysicsScene());
                }
            }
        }

        private Vector3 GetLayoutPosition(Bounds bounds, int envNumber)
        {
            switch (layoutType)
            {
                case LayoutType.Grid:
                    int y = envNumber / maxHorizontalGridSize;
                    int x = envNumber - (maxHorizontalGridSize * y);
                    return new Vector3((bounds.size.x + gridSafetyDistance.x) * x, (bounds.size.y + gridSafetyDistance.y) * -y, 0);
                case LayoutType.Horizontal:
                    return Vector3.right * (bounds.size.x + safetyDistance) * envNumber;
                case LayoutType.Vertical:
                    return Vector3.down * (bounds.size.y + safetyDistance) * envNumber;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        private void FixedUpdate()
        {
            foreach (PhysicsScene spawnedPhysicsScene in spawnedPhysicsScenes)
            {
                spawnedPhysicsScene.Simulate(Time.fixedDeltaTime);
            }
        }
    
        private static Bounds CalculateEnvironmentBounds(GameObject go)
        {
            Bounds bounds = new Bounds();

            foreach (BoxCollider col in go.GetComponentsInChildren<BoxCollider>())
            {
                Bounds b = new Bounds
                {
                    center = col.transform.position,
                    size = new Vector3(
                        col.size.x * col.transform.lossyScale.x,
                        col.size.y * col.transform.lossyScale.y,
                        col.size.z * col.transform.lossyScale.z)
                };
                bounds.Encapsulate(b);
            }

            return bounds;
        }
    }
}