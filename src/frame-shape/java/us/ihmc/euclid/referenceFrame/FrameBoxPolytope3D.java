package us.ihmc.euclid.referenceFrame;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoxPolytope3DView;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractFace3D;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractHalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractVertex3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameBoxPolytope3D implements FrameBoxPolytope3DView
{
   private final FrameBox3D box3D;

   // Faces
   private final Face xMinFace, yMinFace, zMinFace;
   private final Face xMaxFace, yMaxFace, zMaxFace;
   private final List<Face> faces;
   // Edge
   // xMax and xMin
   private final HalfEdge xMinE0, xMinE1, xMinE2, xMinE3, xMaxE0, xMaxE1, xMaxE2, xMaxE3;
   // yMax and yMin
   private final HalfEdge yMinE0, yMinE1, yMinE2, yMinE3, yMaxE0, yMaxE1, yMaxE2, yMaxE3;
   // zMax and zMin
   private final HalfEdge zMinE0, zMinE1, zMinE2, zMinE3, zMaxE0, zMaxE1, zMaxE2, zMaxE3;
   private final List<HalfEdge> edges;
   // Vertices zMin face (clockwise ordering around z-axis)
   private final Vertex v4, v5, v6, v7;
   // Vertices zMax face (clockwise ordering around z-axis)
   private final Vertex v0, v1, v2, v3;
   private final List<Vertex> vertices;

   public FrameBoxPolytope3D(FrameBox3D box3D)
   {
      this.box3D = box3D;
      DoubleSupplier xMin = () -> -0.5 * box3D.getSizeX();
      DoubleSupplier yMin = () -> -0.5 * box3D.getSizeY();
      DoubleSupplier zMin = () -> -0.5 * box3D.getSizeZ();
      DoubleSupplier xMax = () -> +0.5 * box3D.getSizeX();
      DoubleSupplier yMax = () -> +0.5 * box3D.getSizeY();
      DoubleSupplier zMax = () -> +0.5 * box3D.getSizeZ();

      v0 = new Vertex(xMax, yMax, zMax);
      v1 = new Vertex(xMax, yMin, zMax);
      v2 = new Vertex(xMin, yMin, zMax);
      v3 = new Vertex(xMin, yMax, zMax);
      v4 = new Vertex(xMax, yMax, zMin);
      v5 = new Vertex(xMax, yMin, zMin);
      v6 = new Vertex(xMin, yMin, zMin);
      v7 = new Vertex(xMin, yMax, zMin);

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

      DoubleSupplier xFaceAreaSupplier = () -> box3D.getSizeY() * box3D.getSizeZ();
      DoubleSupplier yFaceAreaSupplier = () -> box3D.getSizeX() * box3D.getSizeZ();
      DoubleSupplier zFaceAreaSupplier = () -> box3D.getSizeX() * box3D.getSizeY();

      xMinFace = new Face(Axis3D.X.negated(), xFaceAreaSupplier, xMinE0, xMinE1, xMinE2, xMinE3);
      yMinFace = new Face(Axis3D.Y.negated(), yFaceAreaSupplier, yMinE0, yMinE1, yMinE2, yMinE3);
      zMinFace = new Face(Axis3D.Z.negated(), zFaceAreaSupplier, zMinE0, zMinE1, zMinE2, zMinE3);
      xMaxFace = new Face(Axis3D.X, xFaceAreaSupplier, xMaxE0, xMaxE1, xMaxE2, xMaxE3);
      yMaxFace = new Face(Axis3D.Y, yFaceAreaSupplier, yMaxE0, yMaxE1, yMaxE2, yMaxE3);
      zMaxFace = new Face(Axis3D.Z, zFaceAreaSupplier, zMaxE0, zMaxE1, zMaxE2, zMaxE3);

      faces = Collections.unmodifiableList(Arrays.asList(xMinFace, yMinFace, zMinFace, xMaxFace, yMaxFace, zMaxFace));
      edges = Collections.unmodifiableList(faces.stream().flatMap(f -> f.getEdges().stream()).collect(Collectors.toList()));
      vertices = Collections.unmodifiableList(Arrays.asList(v0, v1, v2, v3, v4, v5, v6, v7));

      box3D.addChangeListeners(faces);
      box3D.addChangeListeners(vertices);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return box3D.getReferenceFrame();
   }

   @Override
   public Face getXMinFace()
   {
      return xMinFace;
   }

   @Override
   public Face getYMinFace()
   {
      return yMinFace;
   }

   @Override
   public Face getZMinFace()
   {
      return zMinFace;
   }

   @Override
   public Face getXMaxFace()
   {
      return xMaxFace;
   }

   @Override
   public Face getYMaxFace()
   {
      return yMaxFace;
   }

   @Override
   public Face getZMaxFace()
   {
      return zMaxFace;
   }

   @Override
   public FrameBox3DReadOnly getOwner()
   {
      return box3D;
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

   private class Face extends AbstractFace3D<Vertex, HalfEdge, Face> implements FrameFace3DReadOnly, Shape3DChangeListener
   {
      private final HalfEdge e0;
      private final HalfEdge e1;
      private final HalfEdge e2;
      private final HalfEdge e3;

      private final FixedFrameBoundingBox3DBasics boundingBox = EuclidFrameFactories.newObservableFixedFrameBoundingBox3DBasics(this,
                                                                                                                                null,
                                                                                                                                axis -> updateBoundingBox());

      private final FixedFramePoint3DBasics centroid = EuclidFrameFactories.newObservableFixedFramePoint3DBasics(this, null, axis -> updateCentroidAndArea());
      private final FixedFrameUnitVector3DBasics normal = EuclidFrameFactories.newObservableFixedFrameUnitVector3DBasics(this, null, axis -> updateNormal());
      private final Vector3DReadOnly normalLocal;
      private final DoubleSupplier areaSupplier;

      private boolean isBoundingBoxDirty = true;
      private boolean isCentroidDirty = true;
      private boolean isNormalDirty = true;

      private Face(Vector3DReadOnly normalLocal, DoubleSupplier areaSupplier, HalfEdge e0, HalfEdge e1, HalfEdge e2, HalfEdge e3)
      {
         super(null, 0.0);

         this.normalLocal = normalLocal;
         this.areaSupplier = areaSupplier;
         this.e0 = e0;
         this.e1 = e1;
         this.e2 = e2;
         this.e3 = e3;
         initialize(Arrays.asList(e0, e1, e2, e3), normalLocal);

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

      @Override
      public void updateBoundingBox()
      {
         if (isBoundingBoxDirty)
         {
            super.updateBoundingBox();
            isBoundingBoxDirty = false;
         }
      }

      @Override
      public void updateCentroidAndArea()
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

      @Override
      public void updateNormal()
      {
         if (isNormalDirty)
         {
            box3D.getPose().inverseTransform(normalLocal, normal);
            isNormalDirty = false;
         }
      }

      @Override
      public FixedFramePoint3DBasics getCentroid()
      {
         return centroid;
      }

      @Override
      public FixedFrameVector3DBasics getNormal()
      {
         return normal;
      }

      @Override
      public double getArea()
      {
         return areaSupplier.getAsDouble();
      }

      @Override
      public FixedFrameBoundingBox3DBasics getBoundingBox()
      {
         return boundingBox;
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return box3D.getReferenceFrame();
      }
   }

   private class HalfEdge extends AbstractHalfEdge3D<Vertex, HalfEdge, Face> implements FrameHalfEdge3DReadOnly
   {
      private HalfEdge(Vertex origin, Vertex destination)
      {
         super(origin, destination);
      }

      void findAndSetTwin()
      {
         HalfEdge edgeTo = getDestination().getEdgeTo(getOrigin());
         if (edgeTo != null)
         {
            setTwin(edgeTo);
            getTwin().setTwin(this);
         }
      }

      @Override
      public Vertex getFirstEndpoint()
      {
         return getOrigin();
      }

      @Override
      public Vertex getSecondEndpoint()
      {
         return getDestination();
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return box3D.getReferenceFrame();
      }
   }

   private class Vertex extends AbstractVertex3D<Vertex, HalfEdge, Face> implements FrameVertex3DReadOnly, Shape3DChangeListener, FixedFramePoint3DBasics
   {
      private final Point3DReadOnly positionLocal;

      private boolean dirty = true;

      private Vertex(DoubleSupplier xLocal, DoubleSupplier yLocal, DoubleSupplier zLocal)
      {
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
            box3D.getPose().inverseTransform(positionLocal, this);
            dirty = false;
         }
      }

      @Override
      public HalfEdge getEdgeTo(FrameVertex3DReadOnly destination)
      {
         return (HalfEdge) FrameVertex3DReadOnly.super.getEdgeTo(destination);
      }

      @Override
      public double getX()
      {
         update();
         return super.getX();
      }

      @Override
      public double getY()
      {
         update();
         return super.getY();
      }

      @Override
      public double getZ()
      {
         update();
         return super.getZ();
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return box3D.getReferenceFrame();
      }
   }
}
