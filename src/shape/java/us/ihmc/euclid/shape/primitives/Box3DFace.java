package us.ihmc.euclid.shape.primitives;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryFactories;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

class Box3DFace implements Face3DReadOnly, Shape3DChangeListener
{
   private final Transform pose;
   private final Box3DHalfEdge e0;
   private final Box3DHalfEdge e1;
   private final Box3DHalfEdge e2;
   private final Box3DHalfEdge e3;
   private final List<Box3DHalfEdge> edges;

   private final Point3D minPoint = new Point3D()
   {
      @Override
      public double getX()
      {
         updateBoundingBox();
         return super.getX();
      };

      @Override
      public double getY()
      {
         updateBoundingBox();
         return super.getY();
      };

      @Override
      public double getZ()
      {
         updateBoundingBox();
         return super.getZ();
      };
   };
   private final Point3D maxPoint = new Point3D()
   {
      @Override
      public double getX()
      {
         updateBoundingBox();
         return super.getX();
      };

      @Override
      public double getY()
      {
         updateBoundingBox();
         return super.getY();
      };

      @Override
      public double getZ()
      {
         updateCentroid();
         return super.getZ();
      };
   };
   private final BoundingBox3DReadOnly boundingBox = EuclidGeometryFactories.newLinkedBoundingBox3D(minPoint, maxPoint);

   private final Point3D centroid = new Point3D()
   {
      @Override
      public double getX()
      {
         updateCentroid();
         return super.getX();
      };

      @Override
      public double getY()
      {
         updateCentroid();
         return super.getY();
      };

      @Override
      public double getZ()
      {
         updateCentroid();
         return super.getZ();
      };
   };
   private final UnitVector3D normal = new UnitVector3D()
   {
      @Override
      public double getRawX()
      {
         updateNormal();
         return super.getRawX();
      };

      @Override
      public double getRawY()
      {
         updateNormal();
         return super.getRawY();
      };

      @Override
      public double getRawZ()
      {
         updateNormal();
         return super.getRawZ();
      };
   };
   private final Vector3DReadOnly normalLocal;

   private double area = 0.0;

   private boolean isBoundingBoxDirty = true;
   private boolean isCentroidDirty = true;
   private boolean isNormalDirty = true;
   private boolean isAreaDirty = true;

   Box3DFace(Transform pose, Vector3DReadOnly normalLocal, Box3DHalfEdge e0, Box3DHalfEdge e1, Box3DHalfEdge e2, Box3DHalfEdge e3)
   {
      this.pose = pose;
      this.normalLocal = normalLocal;
      this.e0 = e0;
      this.e1 = e1;
      this.e2 = e2;
      this.e3 = e3;
      edges = Collections.unmodifiableList(Arrays.asList(e0, e1, e2, e3));

      e0.setNext(e1);
      e1.setNext(e2);
      e2.setNext(e3);
      e3.setNext(e0);

      e0.setPrevious(e3);
      e1.setPrevious(e0);
      e2.setPrevious(e1);
      e3.setPrevious(e2);

      e0.setFace(this);
      e1.setFace(this);
      e2.setFace(this);
      e3.setFace(this);

      e0.findAndSetTwin();
      e1.findAndSetTwin();
      e2.findAndSetTwin();
      e3.findAndSetTwin();
   }

   @Override
   public void changed()
   {
      isBoundingBoxDirty = true;
      isCentroidDirty = true;
      isNormalDirty = true;
      isAreaDirty = true;
   }

   private void updateBoundingBox()
   {
      if (isBoundingBoxDirty)
      {
         minPoint.setX(EuclidCoreTools.min(e0.getFirstEndpointX(), e1.getFirstEndpointX(), e2.getFirstEndpointX(), e3.getFirstEndpointX()));
         minPoint.setY(EuclidCoreTools.min(e0.getFirstEndpointY(), e1.getFirstEndpointY(), e2.getFirstEndpointY(), e3.getFirstEndpointY()));
         minPoint.setZ(EuclidCoreTools.min(e0.getFirstEndpointZ(), e1.getFirstEndpointZ(), e2.getFirstEndpointZ(), e3.getFirstEndpointZ()));
         maxPoint.setX(EuclidCoreTools.max(e0.getFirstEndpointX(), e1.getFirstEndpointX(), e2.getFirstEndpointX(), e3.getFirstEndpointX()));
         maxPoint.setY(EuclidCoreTools.max(e0.getFirstEndpointY(), e1.getFirstEndpointY(), e2.getFirstEndpointY(), e3.getFirstEndpointY()));
         maxPoint.setZ(EuclidCoreTools.max(e0.getFirstEndpointZ(), e1.getFirstEndpointZ(), e2.getFirstEndpointZ(), e3.getFirstEndpointZ()));
         isBoundingBoxDirty = false;
      }
   }

   private void updateCentroid()
   {
      if (isCentroidDirty)
      {
         centroid.add(e0.getOrigin(), e1.getOrigin());
         centroid.add(e2.getOrigin());
         centroid.add(e3.getOrigin());
         centroid.scale(0.25);
         isCentroidDirty = false;
      }
   }

   private void updateNormal()
   {
      if (isNormalDirty)
      {
         pose.inverseTransform(normalLocal, normal);
         isNormalDirty = false;
      }
   }

   @Override
   public Point3DReadOnly getCentroid()
   {
      return centroid;
   }

   @Override
   public Vector3DReadOnly getNormal()
   {
      return normal;
   }

   @Override
   public double getArea()
   {
      if (isAreaDirty)
      {
         area = Math.sqrt(e0.lengthSquared() * e1.lengthSquared());
         isAreaDirty = false;
      }
      return area;
   }

   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public int getNumberOfEdges()
   {
      return 4;
   }

   @Override
   public List<? extends HalfEdge3DReadOnly> getEdges()
   {
      return edges;
   }

}
