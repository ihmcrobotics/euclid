package us.ihmc.euclid.shape.primitives;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryFactories;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class BoxPolytope3D implements ConvexPolytope3DReadOnly
{
   private final Box3D box3D;

   // Faces
   private final Face xMaxFace, yMaxFace, zMaxFace;
   private final Face xMinFace, yMinFace, zMinFace;
   private final List<Face> faces;
   // Edge
   // xMax and xMin
   private final HalfEdge xMinE0, xMinE1, xMinE2, xMinE3, xMaxE0, xMaxE1, xMaxE2, xMaxE3;
   // yMax and yMin
   private final HalfEdge yMinE0, yMinE1, yMinE2, yMinE3, yMaxE0, yMaxE1, yMaxE2, yMaxE3;
   // zMax and zMin
   private final HalfEdge zMinE0, zMinE1, zMinE2, zMinE3, zMaxE0, zMaxE1, zMaxE2, zMaxE3;
   private final List<HalfEdge> edges;
   // Vertices zMax face (clockwise ordering around z-axis)
   private final Vertex v0, v1, v2, v3;
   // Vertices zMin face (clockwise ordering around z-axis)
   private final Vertex v4, v5, v6, v7;
   private final List<Vertex> vertices;

   public BoxPolytope3D(Box3D box3D)
   {
      this.box3D = box3D;
      DoubleSupplier xMin = () -> -0.5 * box3D.getSizeX();
      DoubleSupplier xMax = () -> +0.5 * box3D.getSizeX();
      DoubleSupplier yMin = () -> -0.5 * box3D.getSizeY();
      DoubleSupplier yMax = () -> +0.5 * box3D.getSizeY();
      DoubleSupplier zMin = () -> -0.5 * box3D.getSizeZ();
      DoubleSupplier zMax = () -> +0.5 * box3D.getSizeZ();

      v0 = new Vertex(box3D.getPose(), xMax, yMax, zMax);
      v1 = new Vertex(box3D.getPose(), xMax, yMin, zMax);
      v2 = new Vertex(box3D.getPose(), xMin, yMin, zMax);
      v3 = new Vertex(box3D.getPose(), xMin, yMax, zMax);
      v4 = new Vertex(box3D.getPose(), xMax, yMax, zMin);
      v5 = new Vertex(box3D.getPose(), xMax, yMin, zMin);
      v6 = new Vertex(box3D.getPose(), xMin, yMin, zMin);
      v7 = new Vertex(box3D.getPose(), xMin, yMax, zMin);

      xMaxE0 = new HalfEdge(v0, v4);
      xMaxE1 = new HalfEdge(v4, v5);
      xMaxE2 = new HalfEdge(v5, v1);
      xMaxE3 = new HalfEdge(v1, v0);
      yMaxE0 = new HalfEdge(v0, v3);
      yMaxE1 = new HalfEdge(v3, v7);
      yMaxE2 = new HalfEdge(v7, v4);
      yMaxE3 = new HalfEdge(v4, v0);
      zMaxE0 = new HalfEdge(v0, v1);
      zMaxE1 = new HalfEdge(v1, v2);
      zMaxE2 = new HalfEdge(v2, v3);
      zMaxE3 = new HalfEdge(v3, v0);
      xMinE0 = new HalfEdge(v3, v2);
      xMinE1 = new HalfEdge(v2, v6);
      xMinE2 = new HalfEdge(v6, v7);
      xMinE3 = new HalfEdge(v7, v3);
      yMinE0 = new HalfEdge(v1, v5);
      yMinE1 = new HalfEdge(v5, v6);
      yMinE2 = new HalfEdge(v6, v2);
      yMinE3 = new HalfEdge(v2, v1);
      zMinE0 = new HalfEdge(v4, v7);
      zMinE1 = new HalfEdge(v7, v6);
      zMinE2 = new HalfEdge(v6, v5);
      zMinE3 = new HalfEdge(v5, v4);

      DoubleSupplier xFaceAreaSupplier = () -> box3D.getSizeY() * box3D.getSizeZ();
      DoubleSupplier yFaceAreaSupplier = () -> box3D.getSizeX() * box3D.getSizeZ();
      DoubleSupplier zFaceAreaSupplier = () -> box3D.getSizeX() * box3D.getSizeY();

      xMaxFace = new Face(box3D.getPose(), Axis3D.X, xFaceAreaSupplier, xMaxE0, xMaxE1, xMaxE2, xMaxE3);
      yMaxFace = new Face(box3D.getPose(), Axis3D.Y, yFaceAreaSupplier, yMaxE0, yMaxE1, yMaxE2, yMaxE3);
      zMaxFace = new Face(box3D.getPose(), Axis3D.Z, zFaceAreaSupplier, zMaxE0, zMaxE1, zMaxE2, zMaxE3);
      xMinFace = new Face(box3D.getPose(), Axis3D.X.negated(), xFaceAreaSupplier, xMinE0, xMinE1, xMinE2, xMinE3);
      yMinFace = new Face(box3D.getPose(), Axis3D.Y.negated(), yFaceAreaSupplier, yMinE0, yMinE1, yMinE2, yMinE3);
      zMinFace = new Face(box3D.getPose(), Axis3D.Z.negated(), zFaceAreaSupplier, zMinE0, zMinE1, zMinE2, zMinE3);

      faces = Collections.unmodifiableList(Arrays.asList(xMaxFace, yMaxFace, zMaxFace, xMinFace, yMinFace, zMinFace));
      edges = Collections.unmodifiableList(faces.stream().flatMap(f -> f.getEdges().stream()).collect(Collectors.toList()));
      vertices = Collections.unmodifiableList(Arrays.asList(v0, v1, v2, v3, v4, v5, v6, v7));

      box3D.addChangeListeners(faces);
      box3D.addChangeListeners(vertices);
   }

   @Override
   public List<Face> getFaces()
   {
      return faces;
   }

   @Override
   public List<HalfEdge> getHalfEdges()
   {
      return edges;
   }

   @Override
   public List<Vertex> getVertices()
   {
      return vertices;
   }

   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      return box3D.getBoundingBox();
   }

   @Override
   public Point3DReadOnly getCentroid()
   {
      return box3D.getCentroid();
   }

   @Override
   public double getVolume()
   {
      return box3D.getVolume();
   }

   @Override
   public double getConstructionEpsilon()
   {
      return 0;
   }

   @Override
   public Shape3DBasics copy()
   {
      return null;
   }

   private static class Face implements Face3DReadOnly, Shape3DChangeListener
   {
      private final Transform pose;
      private final HalfEdge e0;
      private final HalfEdge e1;
      private final HalfEdge e2;
      private final HalfEdge e3;
      private final List<HalfEdge> edges;
      private final List<Vertex> vertices;

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
      private final DoubleSupplier areaSupplier;

      private boolean isBoundingBoxDirty = true;
      private boolean isCentroidDirty = true;
      private boolean isNormalDirty = true;

      private Face(Transform pose, Vector3DReadOnly normalLocal, DoubleSupplier areaSupplier, HalfEdge e0, HalfEdge e1, HalfEdge e2, HalfEdge e3)
      {
         this.pose = pose;
         this.normalLocal = normalLocal;
         this.areaSupplier = areaSupplier;
         this.e0 = e0;
         this.e1 = e1;
         this.e2 = e2;
         this.e3 = e3;
         edges = Collections.unmodifiableList(Arrays.asList(e0, e1, e2, e3));
         vertices = Collections.unmodifiableList(Arrays.asList(e0.getOrigin(), e1.getOrigin(), e2.getOrigin(), e3.getOrigin()));

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
         return areaSupplier.getAsDouble();
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
      public List<HalfEdge> getEdges()
      {
         return edges;
      }

      @Override
      public List<Vertex> getVertices()
      {
         return vertices;
      }
   }

   private static class HalfEdge implements HalfEdge3DReadOnly
   {
      private final Vertex origin;
      private final Vertex destination;

      private Face face;
      private HalfEdge previous;
      private HalfEdge next;
      private HalfEdge twin;

      private HalfEdge(Vertex origin, Vertex destination)
      {
         this.origin = origin;
         this.destination = destination;
         origin.addAssociatedEdge(this);
      }

      void setFace(Face face)
      {
         this.face = face;
      }

      void setNext(HalfEdge next)
      {
         this.next = next;
      }

      void setPrevious(HalfEdge previous)
      {
         this.previous = previous;
      }

      void findAndSetTwin()
      {
         twin = destination.getEdgeTo(origin);
         twin.twin = this;
      }

      @Override
      public Vertex getOrigin()
      {
         return origin;
      }

      @Override
      public Vertex getDestination()
      {
         return destination;
      }

      @Override
      public HalfEdge getTwin()
      {
         return twin;
      }

      @Override
      public HalfEdge getNext()
      {
         return next;
      }

      @Override
      public HalfEdge getPrevious()
      {
         return previous;
      }

      @Override
      public Face getFace()
      {
         return face;
      }
   }

   private static class Vertex implements Vertex3DReadOnly, Shape3DChangeListener
   {
      private final List<HalfEdge> associatedEdges = new ArrayList<>();

      private final Point3D position = new Point3D();
      private final Point3DReadOnly positionLocal;
      private final Transform pose;

      private boolean dirty = true;

      private Vertex(Transform pose, DoubleSupplier xLocal, DoubleSupplier yLocal, DoubleSupplier zLocal)
      {
         this.pose = pose;
         positionLocal = EuclidCoreFactories.newLinkedPoint3DReadOnly(xLocal, yLocal, zLocal);
      }

      @Override
      public void changed()
      {
         dirty = true;
      }

      private void update()
      {
         if (dirty)
         {
            pose.inverseTransform(positionLocal, position);
            dirty = false;
         }
      }

      void addAssociatedEdge(HalfEdge associatedEdge)
      {
         associatedEdges.add(associatedEdge);
      }

      @Override
      public HalfEdge getEdgeTo(Vertex3DReadOnly destination)
      {
         return (HalfEdge) Vertex3DReadOnly.super.getEdgeTo(destination);
      }

      @Override
      public double getX()
      {
         update();
         return position.getX();
      }

      @Override
      public double getY()
      {
         update();
         return position.getY();
      }

      @Override
      public double getZ()
      {
         update();
         return position.getZ();
      }

      @Override
      public Collection<HalfEdge> getAssociatedEdges()
      {
         return associatedEdges;
      }

      @Override
      public HalfEdge getAssociatedEdge(int index)
      {
         return associatedEdges.get(index);
      }

      @Override
      public int getNumberOfAssociatedEdges()
      {
         return 3;
      }
   }
}
