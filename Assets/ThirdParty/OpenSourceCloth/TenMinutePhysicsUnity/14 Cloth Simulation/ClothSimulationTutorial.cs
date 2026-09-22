using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClothSimulationTutorial : IGrabbable
{
	private readonly ClothData clothData;

	//Same as in ball physics
	private readonly float[] pos;
	private readonly float[] prevPos;
	private readonly float[] vel;

	//For soft body cloth physics
	//Inverese mass w = 1/m where m is how nuch mass is connected to each particle
	//If a particle is fixed we set its mass to 0
	private readonly float[] invMass;
	//vertex index of the edges that prevent stretching (2 vertices)
	private readonly int[] stretchingIds;
	//vertex index of the edges that prevent bending (4 vertices, the first 2 are the common edge, the last 2 are the vertices the bending edge is going between)
	private readonly int[] bendingIds;
	//The rest length of each edge in the triangulation 
	private readonly float[] stretchingRestLengths;
	//The rest length of each edge that connnects two triangles across the common edge to minimize bending
	private readonly float[] bendingRestLengths;
	// Optional compliant constraints to kinematic world-space attachment points.
	// Unlike SetPinnedParticlePosition(), these particles remain dynamic and can
	// move away from their attachment by a finite spring extension.
	private readonly bool[] springAttachmentEnabled;
	private readonly Vector3[] springAttachmentTargets;
	private readonly float[] springAttachmentRestLengths;
	private readonly float[] springAttachmentCompliances;

	//This array should be global so we don't have to create it a million times
	//Gradients needed when we calculate the edge constraints 
	private readonly float[] grads = new float[4 * 3];
	

	//The Unity mesh to display the cloth
	private Mesh clothMesh;

	//How many vertices (particles) do we have?
	private readonly int numParticles;

	//Simulation settings
	private readonly float[] gravity = { 0f, -9.81f, 0f };
	private readonly int numSubSteps;
	private bool simulate = true;
	private readonly float velocityDamping;
	private readonly bool enableFloorCollision;

	//Soft body behavior settings
	//Compliance (alpha) is the inverse of physical stiffness (k)
	//alpha = 0 means infinitely stiff (hard)
	private readonly float stretchingCompliance;
	private readonly float bendingCompliance;
	private Rigidbody sphereBody;
	private float sphereRadius;
	private float sphereFriction;
	private Vector3 spherePosition;
	private Vector3 sphereVelocity;
	private float sphereInvMass;
	private CapsuleCollider capsuleCollider;
	private Vector3 capsuleCenterOffset;
	private Vector3 capsuleAxis;
	private float capsuleHalfLineLength;


	//Grabbing with mouse to move mesh around

	//The id of the particle we grabed with mouse
	private int grabId = -1;
	//We grab a single particle and then we sit its inverted mass to 0. When we ungrab we have to reset its inverted mass to what itb was before 
	private float grabInvMass = 0f;
	//For custom raycasting
	public List<Vector3> GetMeshVertices => GenerateMeshVertices(this.pos);
	public int[] GetMeshTriangles => clothData.GetFaceTriIds;
	public int GetGrabId => grabId;



	public ClothSimulationTutorial(
		MeshFilter meshFilter,
		ClothData clothData,
		Vector3 startPosOffset,
		float meshScale = 1f,
		float stretchingCompliance = 0f,
		float bendingCompliance = 1f,
		int numSubSteps = 5,
		float velocityDamping = 0f,
		bool enableFloorCollision = true,
		bool pinTutorialRoofCorners = true)
	{
		this.clothData = clothData;
	
		//Particles
		this.numParticles = clothData.GetVerts.Length / 3;

		this.pos = (float[])clothData.GetVerts.Clone();
		//These can start at 0 because they are filled with their correct values after the first iteration 
		this.prevPos = new float[this.pos.Length];
		this.vel = new float[this.pos.Length];
		this.invMass = new float[this.numParticles];
		this.springAttachmentEnabled = new bool[this.numParticles];
		this.springAttachmentTargets = new Vector3[this.numParticles];
		this.springAttachmentRestLengths = new float[this.numParticles];
		this.springAttachmentCompliances = new float[this.numParticles];

		//Give the mesh the correct scale
		for (int i = 0; i < this.pos.Length; i++)
		{
			this.pos[i] *= meshScale;
		}

		//Give the mesh the correct start position
		Translate(startPosOffset.x, startPosOffset.y, startPosOffset.z);


		//Stretching and bending constraints

		//If an edge has a neighbor, the neighbors global edge number is in this list (-1 if has no neighbor)
		int[] neighbors = FindTriNeighbors(clothData.GetFaceTriIds);

		int numTris = clothData.GetFaceTriIds.Length / 3;
		
		List<int> edgeIds = new ();
		List<int> triPairIds = new ();

		//For each triangle
		for (int i = 0; i < numTris; i++)
		{
			//For each edge in the triangle
			for (int j = 0; j < 3; j++)
			{
				int id0 = clothData.GetFaceTriIds[3 * i + j];
				int id1 = clothData.GetFaceTriIds[3 * i + (j + 1) % 3];

				//Global edge number
				int n = neighbors[3 * i + j];

				//Each edge only once
				//Create distance constraint
				if (n < 0 || id0 < id1)
				{
					edgeIds.Add(id0);
					edgeIds.Add(id1);
				}

				//Tri pair
				//Create bending constraint
				if (n >= 0)
				{
					//Opposite ids
					//From global edge number to local edge number 
					int ni = Mathf.FloorToInt(n / 3);
					int nj = n % 3;
					
					int id2 = clothData.GetFaceTriIds[3 * i + (j + 2) % 3];
					int id3 = clothData.GetFaceTriIds[3 * ni + (nj + 2) % 3];
					
					//The vertices of the common edge
					triPairIds.Add(id0);
					triPairIds.Add(id1);
					//The vertices the bending edge is going between
					triPairIds.Add(id2);
					triPairIds.Add(id3);
				}
			}
		}


		this.stretchingIds = edgeIds.ToArray();
		this.bendingIds = triPairIds.ToArray();
		this.stretchingRestLengths = new float[this.stretchingIds.Length / 2];
		this.bendingRestLengths = new float[this.bendingIds.Length / 4];

		this.stretchingCompliance = Mathf.Max(0f, stretchingCompliance);
		this.bendingCompliance = bendingCompliance;
		this.numSubSteps = Mathf.Max(1, numSubSteps);
		this.velocityDamping = Mathf.Max(0f, velocityDamping);
		this.enableFloorCollision = enableFloorCollision;

		//Init the array values
		InitArrays(clothData.GetFaceTriIds, pinTutorialRoofCorners);

		//Init the mesh
		InitMesh(meshFilter, clothData.GetFaceTriIds);
	}



	//Identify triangle neighboring edges - also known as opposite or common edge
	//Explained in the video https://www.youtube.com/watch?v=z5oWopN39OU at 4:00
	//Edges are identified by their global edge number (-1 if has no neighbor)
	//How to use this data?
	//Compute the global edge number: 3 * triNumber + localEdgeNumber where localEdgeNumber is from the vertex the edge is going from in counter-clockwise order
	//What's the opposite edge to t0, e2?
	//globalEdgeNumber = 3 * 0 + 2 = 2 -> neighbors[2] = 4
	//How do we go from global edge number to local edge number? 
	//triangle index = FloorToInt(4 / 3) = 1.3333 = 1
	//edge index = 4 % 3 = 1
	//So opposite edge is t1, e1
	private int[] FindTriNeighbors(int[] triIds)
	{
		//Create a list with all edges
		List<ClothEdge> edges = new ();

		int numTris = triIds.Length / 3;

		//For each triangle
		for (int i = 0; i < numTris; i++)
		{
			//For each vertex in the triangle, create 1 edge going from that vertex to the next vertex in the triangle  
			for (int j = 0; j < 3; j++)
			{
				int id0 = triIds[3 * i + j];
				int id1 = triIds[3 * i + (j + 1) % 3]; //% 3 so the last vertex connects to the first vertex

				int globalEdgeNumber = 3 * i + j;

				edges.Add( new ClothEdge(Mathf.Min(id0, id1), Mathf.Max(id0, id1), globalEdgeNumber));
			}
		}

		//Sort so common edges are next to each other, meaning the edge going from 1 -> 2 is followed by the edge going from 2 -> 1, which is now also going from 1 -> 2 because how we defined the edges
		edges.Sort((a, b) => ((a.id0 < b.id0) || (a.id0 == b.id0 && a.id1 < b.id1)) ? -1 : 1);

		//Find matching edges
		int[] neighbors = new int[triIds.Length];

		//Init all edges to have no neighbors
		System.Array.Fill(neighbors, -1);

		//Find opposite edges
		int nr = 0;

		while (nr < edges.Count)
		{
			ClothEdge e0 = edges[nr];
			
			nr++;
			
			if (nr < edges.Count)
			{
				ClothEdge e1 = edges[nr];

				if (e0.id0 == e1.id0 && e0.id1 == e1.id1)
				{
					neighbors[e0.edgeNr] = e1.edgeNr;
					neighbors[e1.edgeNr] = e0.edgeNr;
				}

				nr++;
			}
		}

		return neighbors;
	}




	private void InitArrays(int[] triIds, bool pinTutorialRoofCorners)
	{
		//Init inverse mass
		//How much mass is connected to a vertex? Use the area of the triangle and divide by 3
		int numTris = triIds.Length / 3;

		float[] e0 = { 0f, 0f, 0f };
		float[] e1 = { 0f, 0f, 0f };
		float[] c  = { 0f, 0f, 0f };

		for (int i = 0; i < numTris; i++)
		{
			//Calculate the area of the triangle
			int id0 = triIds[3 * i];
			int id1 = triIds[3 * i + 1];
			int id2 = triIds[3 * i + 2];

			//a
			VectorArrays.VecSetDiff(e0, 0, this.pos, id1, this.pos, id0);
			//b
			VectorArrays.VecSetDiff(e1, 0, this.pos, id2, this.pos, id0);
			//a x b
			VectorArrays.VecSetCross(c, 0, e0, 0, e1, 0);

			//A = 0.5 * |a x b|
			float A = 0.5f * Mathf.Sqrt(VectorArrays.VecLengthSquared(c, 0));
			
			float pInvMass = A > 0f ? 1f / (A / 3f) : 0f;
			
			this.invMass[id0] += pInvMass;
			this.invMass[id1] += pInvMass;
			this.invMass[id2] += pInvMass;
		}


		//Init stretching lengths
		for (int i = 0; i < this.stretchingRestLengths.Length; i++)
		{
			int id0 = this.stretchingIds[2 * i];
			int id1 = this.stretchingIds[2 * i + 1];
			
			this.stretchingRestLengths[i] = Mathf.Sqrt(VectorArrays.VecDistSquared(this.pos, id0, this.pos, id1));
		}

		//Init bending lengths
		for (int i = 0; i < this.bendingRestLengths.Length; i++)
		{
			//Index 0 and 1 in the array is the common edge, 2 and 3 are the vertices the edge is going between
			int id0 = this.bendingIds[4 * i + 2];
			int id1 = this.bendingIds[4 * i + 3];

			this.bendingRestLengths[i] = Mathf.Sqrt(VectorArrays.VecDistSquared(this.pos, id0, this.pos, id1));
		}


		if (!pinTutorialRoofCorners)
		{
			return;
		}

		//Attach the cloth to the roof so it doesnt fall down (the vertex to the top left and top right)
		float minX = float.MaxValue;
		float maxX = -float.MaxValue;
		float maxY = -float.MaxValue;

		for (int i = 0; i < this.numParticles; i++)
		{
			minX = Mathf.Min(minX, this.pos[3 * i]);
			maxX = Mathf.Max(maxX, this.pos[3 * i]);
			maxY = Mathf.Max(maxY, this.pos[3 * i + 1]);
		}

		float eps = 0.0001f;

		for (int i = 0; i < this.numParticles; i++)
		{
			float x = this.pos[3 * i];
			float y = this.pos[3 * i + 1];

			if ((y > maxY - eps) && (x < minX + eps || x > maxX - eps))
			{
				this.invMass[i] = 0f;
			}
		}
	}



	//
	// Custom Unity methods being called from another script 
	//

	public void MyFixedUpdate()
	{
		if (!simulate)
		{
			return;
		}

		Simulate();
	}

	/// <summary>Turns one mesh particle into a kinematic anchor at a world-space position.</summary>
	public void SetPinnedParticlePosition(int particleId, Vector3 worldPosition)
	{
		if (particleId < 0 || particleId >= numParticles)
		{
			throw new System.ArgumentOutOfRangeException(nameof(particleId));
		}

		float[] p = { worldPosition.x, worldPosition.y, worldPosition.z };
		this.invMass[particleId] = 0f;
		VectorArrays.VecCopy(this.pos, particleId, p, 0);
		VectorArrays.VecCopy(this.prevPos, particleId, p, 0);
		VectorArrays.VecCopy(this.vel, particleId, new float[3], 0);
	}

	/// <summary>
	/// Connects a dynamic cloth particle to a kinematic world-space point using
	/// a compliant distance constraint. A positive rest length models a short
	/// elastic lead between the attachment point and the cloth corner.
	/// </summary>
	public void SetSpringAttachment(
		int particleId,
		Vector3 worldPosition,
		float restLength,
		float compliance)
	{
		if (particleId < 0 || particleId >= numParticles)
		{
			throw new System.ArgumentOutOfRangeException(nameof(particleId));
		}

		springAttachmentEnabled[particleId] = true;
		springAttachmentTargets[particleId] = worldPosition;
		springAttachmentRestLengths[particleId] = Mathf.Max(0f, restLength);
		springAttachmentCompliances[particleId] = Mathf.Max(0f, compliance);
	}

	/// <summary>Overrides the tutorial's default downward gravity for controlled tests.</summary>
	public void SetGravity(Vector3 worldGravity)
	{
		gravity[0] = worldGravity.x;
		gravity[1] = worldGravity.y;
		gravity[2] = worldGravity.z;
	}

	/// <summary>
	/// Scales the inverse mass of every non-pinned cloth particle. This is
	/// useful when a cloth is driven by kinematic anchors: the tutorial's
	/// area-derived default inverse masses are intentionally arbitrary and can
	/// make a contact body push the whole cloth aside instead of being towed.
	/// Call this once during setup, before or after pinning particles.
	/// </summary>
	public void ScaleDynamicParticleInverseMass(float scale)
	{
		scale = Mathf.Max(0.000001f, scale);
		for (int i = 0; i < numParticles; i++)
		{
			if (invMass[i] > 0f)
				invMass[i] *= scale;
		}
	}

	/// <summary>
	/// Enables two-way contact between this XPBD cloth and one Unity sphere
	/// Rigidbody. The contact constraint is solved inside every XPBD substep,
	/// rather than relying on a delayed zero-thickness MeshCollider.
	/// </summary>
	public void SetSphereContact(Rigidbody body, float radius, float friction)
	{
		sphereBody = body;
		sphereRadius = Mathf.Max(0.0001f, radius);
		sphereFriction = Mathf.Max(0f, friction);
		capsuleCollider = null;
	}

	/// <summary>Enables two-way XPBD contact against a Unity CapsuleCollider.</summary>
	public void SetCapsuleContact(Rigidbody body, CapsuleCollider capsule, float friction)
	{
		if (body == null)
			throw new System.ArgumentNullException(nameof(body));
		if (capsule == null)
			throw new System.ArgumentNullException(nameof(capsule));

		sphereBody = body;
		capsuleCollider = capsule;
		sphereFriction = Mathf.Max(0f, friction);
	}



	public void MyUpdate()
	{
		simulate = true;

		
		//Launch the mesh upwards when pressing space
		if (Input.GetKey(KeyCode.Space))
		{
			Yeet();
		}

		if (simulate)
		{
			//Update the visual mesh
			UpdateMesh();
		}
	}

	/// <summary>Publishes the current solver vertices to Unity immediately.</summary>
	public void RefreshMesh()
	{
		UpdateMesh();
	}



	public Mesh MyOnDestroy()
	{
		return clothMesh;
	}



	//
	// Simulation
	//

	//Main cloth simulation loop
	void Simulate()
	{
		float dt = Time.fixedDeltaTime;
		BeginContact();

		float sdt = dt / this.numSubSteps;

		for (int step = 0; step < this.numSubSteps; step++)
		{
			PreSolve(sdt, this.gravity);

			SolveConstraints(sdt);
			SolveContact();

			PostSolve(sdt);
		}

		CommitContact();
	}



	private void PreSolve(float dt, float[] gravity)
	{
		for (int i = 0; i < this.numParticles; i++)
		{
			//The particle is fixed, so don't simulate it
			if (this.invMass[i] == 0f)
			{
				continue;
			}

			//v = v + dt * g
			VectorArrays.VecAdd(this.vel, i, gravity, 0, dt);

			//xPrev = x
			VectorArrays.VecCopy(this.prevPos, i, this.pos, i);

			//x = x + dt * v
			VectorArrays.VecAdd(this.pos, i, this.vel, i, dt);

			//Floor collision
			float y = this.pos[3 * i + 1];

			if (this.enableFloorCollision && y < 0f)
			{
				VectorArrays.VecCopy(this.pos, i, this.prevPos, i);

				this.pos[3 * i + 1] = 0.01f;
			}
		}
	}



	private void SolveConstraints(float dt)
	{
		//Two edge constraints
		SolveStretching(this.stretchingCompliance, dt);
		SolveBending(this.bendingCompliance, dt);
		SolveSpringAttachments(dt);
	}

	private void SolveSpringAttachments(float dt)
	{
		for (int i = 0; i < numParticles; i++)
		{
			if (!springAttachmentEnabled[i] || invMass[i] <= 0f)
				continue;

			Vector3 particlePosition = GetPosition(i);
			Vector3 offset = particlePosition - springAttachmentTargets[i];
			float length = offset.magnitude;
			if (length <= 0.000001f)
				continue;

			float constraint = length - springAttachmentRestLengths[i];
			float alpha = springAttachmentCompliances[i] / (dt * dt);
			float lambda = -constraint / (invMass[i] + alpha);
			AddPosition(i, offset / length * (invMass[i] * lambda));
		}
	}

	private void BeginContact()
	{
		if (sphereBody == null)
			return;
		spherePosition = sphereBody.position;
		sphereVelocity = sphereBody.linearVelocity;
		sphereInvMass = sphereBody.isKinematic || sphereBody.mass <= 0f ? 0f : 1f / sphereBody.mass;
		if (capsuleCollider != null && capsuleCollider.enabled)
			CacheCapsuleGeometry();
	}

	private void CommitContact()
	{
		if (sphereBody == null || sphereInvMass <= 0f)
			return;
		sphereBody.position = spherePosition;
		sphereBody.linearVelocity = sphereVelocity;
		sphereBody.WakeUp();
	}

	private void SolveContact()
	{
		if (sphereBody == null || sphereInvMass <= 0f)
			return;
		if (capsuleCollider != null && capsuleCollider.enabled)
		{
			SolveCapsuleContact();
			return;
		}

		SolveSphereContact();
	}

	private void SolveSphereContact()
	{

		int[] triIds = clothData.GetFaceTriIds;
		for (int tri = 0; tri < triIds.Length; tri += 3)
		{
			int i0 = triIds[tri];
			int i1 = triIds[tri + 1];
			int i2 = triIds[tri + 2];
			Vector3 p0 = GetPosition(i0);
			Vector3 p1 = GetPosition(i1);
			Vector3 p2 = GetPosition(i2);
			Vector3 closest = ClosestPointOnTriangle(spherePosition, p0, p1, p2, out Vector3 barycentric);
			ResolveContact(i0, i1, i2, p0, p1, p2, spherePosition, closest, barycentric, sphereRadius);
		}
	}

	private void SolveCapsuleContact()
	{
		int[] triIds = clothData.GetFaceTriIds;
		for (int tri = 0; tri < triIds.Length; tri += 3)
		{
			Vector3 endpoint0 = spherePosition + capsuleCenterOffset - capsuleAxis * capsuleHalfLineLength;
			Vector3 endpoint1 = spherePosition + capsuleCenterOffset + capsuleAxis * capsuleHalfLineLength;
			int i0 = triIds[tri];
			int i1 = triIds[tri + 1];
			int i2 = triIds[tri + 2];
			Vector3 p0 = GetPosition(i0);
			Vector3 p1 = GetPosition(i1);
			Vector3 p2 = GetPosition(i2);
			ClosestPointsSegmentTriangle(endpoint0, endpoint1, p0, p1, p2,
				out Vector3 capsulePoint, out Vector3 closest, out Vector3 barycentric);
			ResolveContact(i0, i1, i2, p0, p1, p2, capsulePoint, closest, barycentric, sphereRadius);
		}
	}

	private void CacheCapsuleGeometry()
	{
		Transform capsuleTransform = capsuleCollider.transform;
		Vector3 localAxis = capsuleCollider.direction == 0 ? Vector3.right
			: capsuleCollider.direction == 1 ? Vector3.up : Vector3.forward;
		Vector3 scale = capsuleTransform.lossyScale;
		float axisScale = Mathf.Abs(capsuleCollider.direction == 0 ? scale.x
			: capsuleCollider.direction == 1 ? scale.y : scale.z);
		float radialScale = capsuleCollider.direction == 0
			? Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z))
			: capsuleCollider.direction == 1
				? Mathf.Max(Mathf.Abs(scale.x), Mathf.Abs(scale.z))
				: Mathf.Max(Mathf.Abs(scale.x), Mathf.Abs(scale.y));

		capsuleAxis = capsuleTransform.TransformDirection(localAxis).normalized;
		capsuleCenterOffset = capsuleTransform.TransformPoint(capsuleCollider.center) - sphereBody.position;
		sphereRadius = Mathf.Max(0.0001f, capsuleCollider.radius * radialScale);
		float worldHeight = capsuleCollider.height * axisScale;
		capsuleHalfLineLength = Mathf.Max(0f, 0.5f * worldHeight - sphereRadius);
	}

	private static void ClosestPointsSegmentTriangle(
		Vector3 segmentStart, Vector3 segmentEnd, Vector3 a, Vector3 b, Vector3 c,
		out Vector3 segmentPoint, out Vector3 trianglePoint, out Vector3 barycentric)
	{
		Vector3 bestSegmentPoint = segmentStart;
		Vector3 bestTrianglePoint = ClosestPointOnTriangle(segmentStart, a, b, c, out Vector3 bestBarycentric);
		float bestDistanceSquared = (bestSegmentPoint - bestTrianglePoint).sqrMagnitude;

		ConsiderClosestPair(ref bestSegmentPoint, ref bestTrianglePoint, ref bestBarycentric, ref bestDistanceSquared,
			segmentEnd, ClosestPointOnTriangle(segmentEnd, a, b, c, out _), a, b, c);
		ConsiderClosestPair(ref bestSegmentPoint, ref bestTrianglePoint, ref bestBarycentric, ref bestDistanceSquared,
			ClosestPointOnSegment(a, segmentStart, segmentEnd), a, a, b, c);
		ConsiderClosestPair(ref bestSegmentPoint, ref bestTrianglePoint, ref bestBarycentric, ref bestDistanceSquared,
			ClosestPointOnSegment(b, segmentStart, segmentEnd), b, a, b, c);
		ConsiderClosestPair(ref bestSegmentPoint, ref bestTrianglePoint, ref bestBarycentric, ref bestDistanceSquared,
			ClosestPointOnSegment(c, segmentStart, segmentEnd), c, a, b, c);
		ClosestPointsOnSegments(segmentStart, segmentEnd, a, b, out Vector3 onSegment, out Vector3 onEdge);
		ConsiderClosestPair(ref bestSegmentPoint, ref bestTrianglePoint, ref bestBarycentric, ref bestDistanceSquared, onSegment, onEdge, a, b, c);
		ClosestPointsOnSegments(segmentStart, segmentEnd, b, c, out onSegment, out onEdge);
		ConsiderClosestPair(ref bestSegmentPoint, ref bestTrianglePoint, ref bestBarycentric, ref bestDistanceSquared, onSegment, onEdge, a, b, c);
		ClosestPointsOnSegments(segmentStart, segmentEnd, c, a, out onSegment, out onEdge);
		ConsiderClosestPair(ref bestSegmentPoint, ref bestTrianglePoint, ref bestBarycentric, ref bestDistanceSquared, onSegment, onEdge, a, b, c);

		Vector3 normal = Vector3.Cross(b - a, c - a);
		float normalLength = normal.magnitude;
		if (normalLength <= 0.000001f)
		{
			segmentPoint = bestSegmentPoint;
			trianglePoint = bestTrianglePoint;
			barycentric = bestBarycentric;
			return;
		}
		normal /= normalLength;
		float startDistance = Vector3.Dot(segmentStart - a, normal);
		float endDistance = Vector3.Dot(segmentEnd - a, normal);
		float denominator = startDistance - endDistance;
		if (Mathf.Abs(denominator) > 0.000001f)
		{
			float t = startDistance / denominator;
			if (t >= 0f && t <= 1f)
			{
				Vector3 intersection = Vector3.Lerp(segmentStart, segmentEnd, t);
				Vector3 projected = ClosestPointOnTriangle(intersection, a, b, c, out Vector3 projectedBarycentric);
				if ((projected - intersection).sqrMagnitude <= 1e-10f)
				{
					segmentPoint = intersection;
					trianglePoint = intersection;
					barycentric = projectedBarycentric;
					return;
				}
			}
		}
		else if (TryFindCoplanarProjectionIntersection(segmentStart, segmentEnd, a, b, c, normal,
			startDistance, out Vector3 projectedSegmentPoint, out Vector3 projectedTrianglePoint, out Vector3 projectedBarycentric))
		{
			bestSegmentPoint = projectedSegmentPoint;
			bestTrianglePoint = projectedTrianglePoint;
			bestBarycentric = projectedBarycentric;
		}
		segmentPoint = bestSegmentPoint;
		trianglePoint = bestTrianglePoint;
		barycentric = bestBarycentric;
	}

	private static void ConsiderClosestPair(
		ref Vector3 segmentPoint, ref Vector3 trianglePoint, ref Vector3 barycentric, ref float bestDistanceSquared,
		Vector3 candidateSegmentPoint, Vector3 candidateTrianglePoint, Vector3 a, Vector3 b, Vector3 c)
	{
		float distanceSquared = (candidateSegmentPoint - candidateTrianglePoint).sqrMagnitude;
		if (distanceSquared >= bestDistanceSquared)
			return;
		bestDistanceSquared = distanceSquared;
		segmentPoint = candidateSegmentPoint;
		trianglePoint = ClosestPointOnTriangle(candidateTrianglePoint, a, b, c, out barycentric);
	}

	private static Vector3 ClosestPointOnSegment(Vector3 point, Vector3 start, Vector3 end)
	{
		Vector3 edge = end - start;
		float denominator = edge.sqrMagnitude;
		if (denominator <= 1e-10f)
			return start;
		return start + edge * Mathf.Clamp01(Vector3.Dot(point - start, edge) / denominator);
	}

	private static void ClosestPointsOnSegments(Vector3 p0, Vector3 p1, Vector3 q0, Vector3 q1,
		out Vector3 onP, out Vector3 onQ)
	{
		Vector3 d1 = p1 - p0;
		Vector3 d2 = q1 - q0;
		Vector3 r = p0 - q0;
		float a = Vector3.Dot(d1, d1);
		float e = Vector3.Dot(d2, d2);
		float f = Vector3.Dot(d2, r);
		float s;
		float t;
		if (a <= 1e-10f && e <= 1e-10f)
		{
			onP = p0;
			onQ = q0;
			return;
		}
		if (a <= 1e-10f)
		{
			s = 0f;
			t = Mathf.Clamp01(f / e);
		}
		else
		{
			float c = Vector3.Dot(d1, r);
			if (e <= 1e-10f)
			{
				t = 0f;
				s = Mathf.Clamp01(-c / a);
			}
			else
			{
				float b = Vector3.Dot(d1, d2);
				float denominator = a * e - b * b;
				s = denominator > 1e-10f ? Mathf.Clamp01((b * f - c * e) / denominator) : 0f;
				t = (b * s + f) / e;
				if (t < 0f)
				{
					t = 0f;
					s = Mathf.Clamp01(-c / a);
				}
				else if (t > 1f)
				{
					t = 1f;
					s = Mathf.Clamp01((b - c) / a);
				}
			}
		}
		onP = p0 + d1 * s;
		onQ = q0 + d2 * t;
	}

	private static bool TryFindCoplanarProjectionIntersection(
		Vector3 segmentStart, Vector3 segmentEnd, Vector3 a, Vector3 b, Vector3 c, Vector3 normal,
		float planeOffset, out Vector3 projectedSegmentPoint, out Vector3 projectedTrianglePoint, out Vector3 barycentric)
	{
		int droppedAxis = Mathf.Abs(normal.x) > Mathf.Abs(normal.y)
			? (Mathf.Abs(normal.x) > Mathf.Abs(normal.z) ? 0 : 2)
			: (Mathf.Abs(normal.y) > Mathf.Abs(normal.z) ? 1 : 2);
		Vector2 s0 = ProjectTo2D(segmentStart, droppedAxis);
		Vector2 s1 = ProjectTo2D(segmentEnd, droppedAxis);
		Vector2 t0 = ProjectTo2D(a, droppedAxis);
		Vector2 t1 = ProjectTo2D(b, droppedAxis);
		Vector2 t2 = ProjectTo2D(c, droppedAxis);
		float segmentT = -1f;
		if (PointInTriangle2D(s0, t0, t1, t2))
			segmentT = 0f;
		else if (PointInTriangle2D(s1, t0, t1, t2))
			segmentT = 1f;
		else
		{
			TrySegmentIntersection2D(s0, s1, t0, t1, ref segmentT);
			TrySegmentIntersection2D(s0, s1, t1, t2, ref segmentT);
			TrySegmentIntersection2D(s0, s1, t2, t0, ref segmentT);
		}

		if (segmentT < 0f)
		{
			projectedSegmentPoint = default;
			projectedTrianglePoint = default;
			barycentric = default;
			return false;
		}

		projectedSegmentPoint = Vector3.Lerp(segmentStart, segmentEnd, segmentT);
		projectedTrianglePoint = projectedSegmentPoint - normal * planeOffset;
		ClosestPointOnTriangle(projectedTrianglePoint, a, b, c, out barycentric);
		return true;
	}

	private static Vector2 ProjectTo2D(Vector3 point, int droppedAxis) => droppedAxis == 0
		? new Vector2(point.y, point.z) : droppedAxis == 1 ? new Vector2(point.x, point.z) : new Vector2(point.x, point.y);

	private static bool PointInTriangle2D(Vector2 point, Vector2 a, Vector2 b, Vector2 c)
	{
		float ab = Cross2D(b - a, point - a);
		float bc = Cross2D(c - b, point - b);
		float ca = Cross2D(a - c, point - c);
		return (ab >= -1e-6f && bc >= -1e-6f && ca >= -1e-6f)
			|| (ab <= 1e-6f && bc <= 1e-6f && ca <= 1e-6f);
	}

	private static void TrySegmentIntersection2D(Vector2 p0, Vector2 p1, Vector2 q0, Vector2 q1, ref float bestT)
	{
		Vector2 r = p1 - p0;
		Vector2 s = q1 - q0;
		float denominator = Cross2D(r, s);
		if (Mathf.Abs(denominator) <= 1e-8f)
			return;
		Vector2 qMinusP = q0 - p0;
		float t = Cross2D(qMinusP, s) / denominator;
		float u = Cross2D(qMinusP, r) / denominator;
		if (t >= 0f && t <= 1f && u >= 0f && u <= 1f && (bestT < 0f || t < bestT))
			bestT = t;
	}

	private static float Cross2D(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;

	private void ResolveContact(
		int i0, int i1, int i2, Vector3 p0, Vector3 p1, Vector3 p2,
		Vector3 bodyContactPoint, Vector3 closest, Vector3 barycentric, float contactRadius)
	{
			Vector3 separation = bodyContactPoint - closest;
			float distance = separation.magnitude;
			if (distance >= contactRadius)
				return;

			Vector3 normal;
			if (distance > 0.00001f)
				normal = separation / distance;
			else
			{
				normal = Vector3.Cross(p1 - p0, p2 - p0).normalized;
				if (normal.sqrMagnitude < 1e-8f)
					return;
				if (Vector3.Dot(sphereVelocity, normal) > 0f)
					normal = -normal;
			}

			float w0 = invMass[i0];
			float w1 = invMass[i1];
			float w2 = invMass[i2];
			float denominator = sphereInvMass + w0 * barycentric.x * barycentric.x
				+ w1 * barycentric.y * barycentric.y + w2 * barycentric.z * barycentric.z;
			if (denominator <= 1e-8f)
				return;

			float lambda = (contactRadius - distance) / denominator;
			spherePosition += normal * (sphereInvMass * lambda);
			AddPosition(i0, -normal * (w0 * barycentric.x * lambda));
			AddPosition(i1, -normal * (w1 * barycentric.y * lambda));
			AddPosition(i2, -normal * (w2 * barycentric.z * lambda));

			Vector3 clothVelocity = GetVelocity(i0) * barycentric.x + GetVelocity(i1) * barycentric.y + GetVelocity(i2) * barycentric.z;
			Vector3 relativeVelocity = sphereVelocity - clothVelocity;
			float closingSpeed = Vector3.Dot(relativeVelocity, normal);
			if (closingSpeed >= 0f)
				return;

			float normalImpulse = -closingSpeed / denominator;
			ApplyVelocityImpulse(i0, i1, i2, barycentric, normal * normalImpulse);

			Vector3 tangentVelocity = relativeVelocity - normal * closingSpeed;
			float tangentSpeed = tangentVelocity.magnitude;
			if (tangentSpeed > 0.00001f && sphereFriction > 0f)
			{
				float maxFrictionImpulse = sphereFriction * normalImpulse;
				float tangentImpulse = Mathf.Min(maxFrictionImpulse, tangentSpeed / denominator);
				ApplyVelocityImpulse(i0, i1, i2, barycentric, -tangentVelocity / tangentSpeed * tangentImpulse);
			}
	}

	private void ApplyVelocityImpulse(int i0, int i1, int i2, Vector3 barycentric, Vector3 impulse)
	{
		sphereVelocity += impulse * sphereInvMass;
		AddVelocity(i0, -impulse * (invMass[i0] * barycentric.x));
		AddVelocity(i1, -impulse * (invMass[i1] * barycentric.y));
		AddVelocity(i2, -impulse * (invMass[i2] * barycentric.z));
	}

	private Vector3 GetPosition(int index) => new Vector3(pos[3 * index], pos[3 * index + 1], pos[3 * index + 2]);
	private Vector3 GetVelocity(int index) => new Vector3(vel[3 * index], vel[3 * index + 1], vel[3 * index + 2]);
	private void AddPosition(int index, Vector3 delta)
	{
		pos[3 * index] += delta.x;
		pos[3 * index + 1] += delta.y;
		pos[3 * index + 2] += delta.z;
	}
	private void AddVelocity(int index, Vector3 delta)
	{
		vel[3 * index] += delta.x;
		vel[3 * index + 1] += delta.y;
		vel[3 * index + 2] += delta.z;
	}

	private static Vector3 ClosestPointOnTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c, out Vector3 barycentric)
	{
		Vector3 ab = b - a;
		Vector3 ac = c - a;
		Vector3 ap = point - a;
		float d1 = Vector3.Dot(ab, ap);
		float d2 = Vector3.Dot(ac, ap);
		if (d1 <= 0f && d2 <= 0f) { barycentric = new Vector3(1f, 0f, 0f); return a; }

		Vector3 bp = point - b;
		float d3 = Vector3.Dot(ab, bp);
		float d4 = Vector3.Dot(ac, bp);
		if (d3 >= 0f && d4 <= d3) { barycentric = new Vector3(0f, 1f, 0f); return b; }

		float vc = d1 * d4 - d3 * d2;
		if (vc <= 0f && d1 >= 0f && d3 <= 0f)
		{
			float v = d1 / (d1 - d3);
			barycentric = new Vector3(1f - v, v, 0f);
			return a + v * ab;
		}

		Vector3 cp = point - c;
		float d5 = Vector3.Dot(ab, cp);
		float d6 = Vector3.Dot(ac, cp);
		if (d6 >= 0f && d5 <= d6) { barycentric = new Vector3(0f, 0f, 1f); return c; }

		float vb = d5 * d2 - d1 * d6;
		if (vb <= 0f && d2 >= 0f && d6 <= 0f)
		{
			float w = d2 / (d2 - d6);
			barycentric = new Vector3(1f - w, 0f, w);
			return a + w * ac;
		}

		float va = d3 * d6 - d5 * d4;
		if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
		{
			Vector3 bc = c - b;
			float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			barycentric = new Vector3(0f, 1f - w, w);
			return b + w * bc;
		}

		float denominator = 1f / (va + vb + vc);
		float vFace = vb * denominator;
		float wFace = vc * denominator;
		barycentric = new Vector3(1f - vFace - wFace, vFace, wFace);
		return a + ab * vFace + ac * wFace;
	}



	private void PostSolve(float dt)
	{
		//For each particle
		for (int i = 0; i < this.numParticles; i++)
		{
			if (this.invMass[i] == 0f)
			{
				continue;
			}

			//Fix velocity
			//v = (x - xPrev) / dt
			VectorArrays.VecSetDiff(this.vel, i, this.pos, i, this.prevPos, i, 1f / dt);
			float damping = Mathf.Clamp01(1f - this.velocityDamping * dt);
			this.vel[3 * i] *= damping;
			this.vel[3 * i + 1] *= damping;
			this.vel[3 * i + 2] *= damping;
		}
	}



	//Same as distance constraint in soft body physics, making each edge in the triangulation move towards its rest length
	private void SolveStretching(float compliance, float dt)
	{
		float alpha = compliance / (dt * dt);

		//For each edge
		for (var i = 0; i < this.stretchingRestLengths.Length; i++)
		{
			//2 vertices per edge in the data structure, so multiply by 2 to get the correct vertex index
			int id0 = this.stretchingIds[2 * i];
			int id1 = this.stretchingIds[2 * i + 1];

			float w0 = this.invMass[id0];
			float w1 = this.invMass[id1];

			float wTot = w0 + w1;

			if (wTot == 0f)
			{
				continue;
			}

			//The current length of the edge l

			//x0-x1
			//The result is stored in grads array
			VectorArrays.VecSetDiff(this.grads, 0, this.pos, id0, this.pos, id1);

			//sqrMargnitude(x0-x1)
			float lSqr = VectorArrays.VecLengthSquared(this.grads, 0);

			float l = Mathf.Sqrt(lSqr);

			//If they are at the same pos we get a divisio by 0 later so ignore
			if (l == 0f)
			{
				continue;
			}

			//(xo-x1) * (1/|x0-x1|) = gradC
			VectorArrays.VecScale(this.grads, 0, 1f / l);

			float l_rest = this.stretchingRestLengths[i];

			float C = l - l_rest;

			//lambda because |grad_Cn|^2 = 1 because if we move a particle 1 unit, the distance between the particles also grows with 1 unit, and w = w0 + w1
			float lambda = -C / (wTot + alpha);

			//Move the vertices x = x + deltaX where deltaX = lambda * w * gradC
			VectorArrays.VecAdd(this.pos, id0, this.grads, 0, lambda * w0);
			VectorArrays.VecAdd(this.pos, id1, this.grads, 0, -lambda * w1);
		}
	}



	//Similar to how stretching constraints work 
	//The only difference is how we identify which vertices are part of the edge 
	private void SolveBending(float compliance, float dt)
	{
		float alpha = compliance / (dt * dt);

		//For each edge
		for (int i = 0; i < this.bendingRestLengths.Length; i++)
		{
			//2 vertices per edge, but this edge is going between two vertices opposite of each other in two triangles, crossing the common edge 
			int id0 = this.bendingIds[4 * i + 2];
			int id1 = this.bendingIds[4 * i + 3];
			
			float w0 = this.invMass[id0];
			float w1 = this.invMass[id1];
			
			float wTot = w0 + w1;
			
			if (wTot == 0f)
			{
				continue;
			}

			//The current length of the edge l

			//x0-x1
			//The result is stored in grads array
			VectorArrays.VecSetDiff(this.grads, 0, this.pos, id0, this.pos, id1);

			//sqrMargnitude(x0-x1)
			float lSqr = VectorArrays.VecLengthSquared(this.grads, 0);

			float l = Mathf.Sqrt(lSqr);

			//If they are at the same pos we get a divisio by 0 later so ignore
			if (l == 0f)
			{
				continue;
			}

			//(xo-x1) * (1/|x0-x1|) = gradC
			VectorArrays.VecScale(this.grads, 0, 1f / l);

			float l_rest = this.bendingRestLengths[i];

			float C = l - l_rest;

			//lambda because |grad_Cn|^2 = 1 because if we move a particle 1 unit, the distance between the particles also grows with 1 unit, and w = w0 + w1
			float lambda = -C / (wTot + alpha);

			//Move the vertices x = x + deltaX where deltaX = lambda * w * gradC
			VectorArrays.VecAdd(this.pos, id0, this.grads, 0, lambda * w0);
			VectorArrays.VecAdd(this.pos, id1, this.grads, 0, -lambda * w1);
		}
	}



	//
	// Unity mesh 
	//

	//Init the mesh when the simulation is started
	private void InitMesh(MeshFilter meshFilter, int[] triangles)
	{
		Mesh mesh = new();

		List<Vector3> vertices = GenerateMeshVertices(this.pos);

		mesh.SetVertices(vertices);
		mesh.triangles = triangles;

		mesh.RecalculateBounds();
		mesh.RecalculateNormals();

		meshFilter.sharedMesh = mesh;

		this.clothMesh = meshFilter.sharedMesh;
		this.clothMesh.MarkDynamic();
	}

	//Update the mesh with new vertex positions
	private void UpdateMesh()
	{
		List<Vector3> vertices = GenerateMeshVertices(this.pos);

		this.clothMesh.SetVertices(vertices);

		this.clothMesh.RecalculateBounds();
		this.clothMesh.RecalculateNormals();
	}

	//Generate the List of vertices needed for a Unity mesh
	private List<Vector3> GenerateMeshVertices(float[] pos)
	{
		List<Vector3> vertices = new();

		for (int i = 0; i < pos.Length; i += 3)
		{
			Vector3 v = new(pos[i], pos[i + 1], pos[i + 2]);

			vertices.Add(v);
		}

		return vertices;
	}



	//
	// Help methods
	//

	//Move all vertices a distance of (x, y, z)
	private void Translate(float x, float y, float z)
	{
		float[] moveDist = new float[] { x, y, z };

		for (int i = 0; i < this.numParticles; i++)
		{
			VectorArrays.VecAdd(this.pos, i, moveDist, 0);
			VectorArrays.VecAdd(this.prevPos, i, moveDist, 0);
		}
	}



	//
	// Mesh user interactions
	//


	//Yeet the mesh upwards
	private void Yeet()
	{
		for (int i = 0; i < this.numParticles; i++)
		{
			//Dont move the fixed particles
			if (invMass[i] == 0f)
			{
				continue;
            }
		
			//Add constant to y coordinate
			this.pos[3 * i + 1] += 0.1f;
		}
	}



	//Input pos is the pos in a triangle we get when doing ray-triangle intersection
	public void StartGrab(Vector3 triangleIntersectionPos)
	{
		float[] p = new float[] { triangleIntersectionPos.x, triangleIntersectionPos.y, triangleIntersectionPos.z };

		//Find the closest vertex to the pos on a triangle in the mesh
		float minD2 = float.MaxValue;

		this.grabId = -1;

		for (int i = 0; i < this.numParticles; i++)
		{
			float d2 = VectorArrays.VecDistSquared(p, 0, this.pos, i);

			if (d2 < minD2)
			{
				minD2 = d2;
				this.grabId = i;
			}
		}

		//We have found a vertex
		if (this.grabId >= 0)
		{
			//Save the current innverted mass
			this.grabInvMass = this.invMass[this.grabId];

			//Set the inverted mass to 0 to mark it as fixed
			this.invMass[this.grabId] = 0f;

			//Set the position of the vertex to the position where the ray hit the triangle
			VectorArrays.VecCopy(this.pos, this.grabId, p, 0);
		}
	}



	public void MoveGrabbed(Vector3 newPos)
	{
		if (this.grabId >= 0)
		{
			float[] p = new float[] { newPos.x, newPos.y, newPos.z };

			VectorArrays.VecCopy(this.pos, this.grabId, p, 0);
		}
	}



	public void EndGrab(Vector3 newPos, Vector3 vel)
	{
		if (this.grabId >= 0)
		{
			//Set the mass to whatever mass it was before we grabbed it
			this.invMass[this.grabId] = this.grabInvMass;

			float[] v = new float[] { vel.x, vel.y, vel.z };

			VectorArrays.VecCopy(this.vel, this.grabId, v, 0);
		}

		this.grabId = -1;
	}



	public void IsRayHittingBody(Ray ray, out CustomHit hit)
	{
		//Mesh data
		Vector3[] vertices = GetMeshVertices.ToArray();

		int[] triangles = GetMeshTriangles;

		//Find if the ray hit a triangle in the mesh
		Intersections.IsRayHittingMesh(ray, vertices, triangles, out hit);
	}



	public Vector3 GetGrabbedPos()
	{
		Vector3 grabbedPos = new Vector3(pos[3 * grabId + 0], pos[3 * grabId + 1], pos[3 * grabId + 2]);

		return grabbedPos;
	}
}
