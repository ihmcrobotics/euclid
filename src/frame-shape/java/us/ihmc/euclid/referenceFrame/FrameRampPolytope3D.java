package us.ihmc.euclid.referenceFrame;

import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories.newObservableFixedFrameBoundingBox3DBasics;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories.newObservableFixedFramePoint3DBasics;
import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories.newObservableFixedFrameVector3DBasics;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newLinkedPoint3DReadOnly;
import static us.ihmc.euclid.tools.EuclidCoreFactories.newNegativeLinkedVector3D;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRampPolytope3DView;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractFace3D;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractHalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractVertex3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a polytope view for a ramp.
 * <p>
 * Do not instantiate directly this class, instead use
 * {@link FrameRamp3DReadOnly#asConvexPolytope()}.
 * </p>
 * <p>
 * This implementation registers listener to its owner such that it is always accurately
 * representing its owner even being modified after creation of the polytope view.
 * </p>
 *
 * @author Sylvain Bertrand
 */
class FrameRampPolytope3D implements FrameRampPolytope3DView
{
   private final FrameRamp3D ramp3D;

   // Faces
   private final Face rampFace;
   private final Face xMaxFace, yMinFace, yMaxFace, zMinFace;
   private final List<Face> faces;
   private final List<HalfEdge> edges;
   private final List<Vertex> vertices;

   FrameRampPolytope3D(FrameRamp3D ramp3D)
   {
      this.ramp3D = ramp3D;
      DoubleSupplier xMin = () -> 0.0, yMin = () -> -0.5 * ramp3D.getSizeY(), zMin = () -> 0.0;
      DoubleSupplier xMax = () -> ramp3D.getSizeX(), yMax = () -> +0.5 * ramp3D.getSizeY(), zMax = () -> ramp3D.getSizeZ();

      Vertex v0 = new Vertex(xMin, yMax, zMin), v1 = new Vertex(xMax, yMax, zMax), v2 = new Vertex(xMax, yMin, zMax);
      Vertex v3 = new Vertex(xMin, yMin, zMin), v4 = new Vertex(xMax, yMax, zMin), v5 = new Vertex(xMax, yMin, zMin);

      HalfEdge rampE0 = new HalfEdge(v0, v1), rampE1 = new HalfEdge(v1, v2), rampE2 = new HalfEdge(v2, v3), rampE3 = new HalfEdge(v3, v0);
      HalfEdge yMinE0 = new HalfEdge(v3, v2), yMinE1 = new HalfEdge(v2, v5), yMinE2 = new HalfEdge(v5, v3);
      HalfEdge yMaxE0 = new HalfEdge(v0, v4), yMaxE1 = new HalfEdge(v4, v1), yMaxE2 = new HalfEdge(v1, v0);
      HalfEdge xMaxE0 = new HalfEdge(v1, v4), xMaxE1 = new HalfEdge(v4, v5), xMaxE2 = new HalfEdge(v5, v2), xMaxE3 = new HalfEdge(v2, v1);
      HalfEdge zMinE0 = new HalfEdge(v0, v3), zMinE1 = new HalfEdge(v3, v5), zMinE2 = new HalfEdge(v5, v4), zMinE3 = new HalfEdge(v4, v0);

      DoubleSupplier slopeFaceAreaSupplier = () -> ramp3D.getSizeY() * ramp3D.getRampLength();
      DoubleSupplier xFaceAreaSupplier = () -> ramp3D.getSizeY() * ramp3D.getSizeZ();
      DoubleSupplier yFaceAreaSupplier = () -> 0.5 * ramp3D.getSizeX() * ramp3D.getSizeZ();
      DoubleSupplier zFaceAreaSupplier = () -> ramp3D.getSizeX() * ramp3D.getSizeY();

      FixedFrameShape3DPoseBasics pose = ramp3D.getPose();

      rampFace = new Face("ramp", ramp3D.getRampSurfaceNormal(), slopeFaceAreaSupplier, rampE0, rampE1, rampE2, rampE3);
      yMinFace = new Face("y-min", newNegativeLinkedVector3D(pose.getYAxis()), yFaceAreaSupplier, yMinE0, yMinE1, yMinE2);
      zMinFace = new Face("z-min", newNegativeLinkedVector3D(pose.getZAxis()), zFaceAreaSupplier, zMinE0, zMinE1, zMinE2, zMinE3);
      xMaxFace = new Face("x-max", pose.getXAxis(), xFaceAreaSupplier, xMaxE0, xMaxE1, xMaxE2, xMaxE3);
      yMaxFace = new Face("y-max", pose.getYAxis(), yFaceAreaSupplier, yMaxE0, yMaxE1, yMaxE2);

      faces = Collections.unmodifiableList(Arrays.asList(rampFace, yMinFace, zMinFace, xMaxFace, yMaxFace));
      edges = Collections.unmodifiableList(faces.stream().flatMap(f -> f.getEdges().stream()).collect(Collectors.toList()));
      vertices = Collections.unmodifiableList(Arrays.asList(v0, v1, v2, v3, v4, v5));

      ramp3D.addChangeListeners(vertices);
      ramp3D.addChangeListeners(faces);
   }

   @Override
   public Face getRampFace()
   {
      return rampFace;
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
   public FrameRamp3DReadOnly getOwner()
   {
      return ramp3D;
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
   public ReferenceFrame getReferenceFrame()
   {
      return ramp3D.getReferenceFrame();
   }

   private class Face extends AbstractFace3D<Vertex, HalfEdge, Face> implements FrameFace3DReadOnly, Shape3DChangeListener
   {
      private final String name;

      private final FixedFrameBoundingBox3DBasics boundingBox = newObservableFixedFrameBoundingBox3DBasics(this, null, (axis, bound) -> updateBoundingBox());
      private final FixedFramePoint3DBasics centroid = newObservableFixedFramePoint3DBasics(this, null, axis -> updateCentroidAndArea());
      private final Vector3DReadOnly normalLocal;
      private final FixedFrameVector3DBasics normal = newObservableFixedFrameVector3DBasics(this, null, axis -> updateNormal());
      private final DoubleSupplier areaSupplier;

      private boolean isBoundingBoxDirty = true;
      private boolean isCentroidDirty = true;
      private boolean isNormalDirty = true;

      private Face(String name, Vector3DReadOnly normalLocal, DoubleSupplier areaSupplier, HalfEdge... edges)
      {
         super(null, 0.0);
         this.name = name;

         this.normalLocal = normalLocal;
         this.areaSupplier = areaSupplier;
         initialize(Arrays.asList(edges), normalLocal);

         for (HalfEdge edge : edges)
            edge.findAndSetTwin();
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
            isBoundingBoxDirty = false;
            super.updateBoundingBox();
         }
      }

      @Override
      public void updateCentroidAndArea()
      {
         if (isCentroidDirty)
         {
            isCentroidDirty = false;
            centroid.setToZero();
            for (int i = 0; i < getNumberOfEdges(); i++)
               centroid.add(getVertex(i));
            centroid.scale(1.0 / getNumberOfEdges());
         }
      }

      @Override
      public void updateNormal()
      {
         if (isNormalDirty)
         {
            isNormalDirty = false;
            normal.set(normalLocal);
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
         return ramp3D.getReferenceFrame();
      }

      @Override
      public String toString()
      {
         return "Ramp3D Face " + name + " " + super.toString();
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
         return ramp3D.getReferenceFrame();
      }
   }

   private class Vertex extends AbstractVertex3D<Vertex, HalfEdge, Face> implements FrameVertex3DReadOnly, Shape3DChangeListener
   {
      private final Point3DReadOnly positionLocal;

      private boolean dirty = true;

      private Vertex(DoubleSupplier xLocal, DoubleSupplier yLocal, DoubleSupplier zLocal)
      {
         positionLocal = newLinkedPoint3DReadOnly(xLocal, yLocal, zLocal);
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
            dirty = false;
            ramp3D.getPose().transform(positionLocal, this);
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
         return ramp3D.getReferenceFrame();
      }
   }
}
