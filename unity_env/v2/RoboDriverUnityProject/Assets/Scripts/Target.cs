using UnityEngine;
using Random = UnityEngine.Random;

public class Target : MonoBehaviour
{
    private const float AREA_HALF_SIZE = 25f;
    
    private void OnEnable()
    {
        ResetTarget();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.TryGetComponent(out RoboDriverAgent driverAgent))
        {
            driverAgent.TargetCollected();
            ResetTarget();
        }
    }

    public void ResetTarget()
    {
        Vector3 offset = new Vector3(
            Random.Range(-AREA_HALF_SIZE, AREA_HALF_SIZE),
            1.5f,
            Random.Range(-AREA_HALF_SIZE, AREA_HALF_SIZE));

        transform.localPosition = offset;
    }
}
