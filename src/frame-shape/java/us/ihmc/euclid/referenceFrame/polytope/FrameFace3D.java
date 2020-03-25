package us.ihmc.euclid.referenceFrame.polytope;

import java.util.Collection;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractFace3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DFactory;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameFace3D extends AbstractFace3D<FrameVertex3D, FrameHalfEdge3D, FrameFace3D> implements FrameFace3DReadOnly, Clearable, Transformable
{
   private final ReferenceFrameHolder referenceFrameHolder;
   /** The normal vector of the support plane of this face. */
   private final FixedFrameVector3DBasics normal = EuclidFrameFactories.newFixedFrameVector3DBasics(this);
   /** The centroid of this face. */
   private final FixedFramePoint3DBasics centroid = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** The tightest bounding box entirely containing this face. */
   private final FixedFrameBoundingBox3DBasics boundingBox = EuclidFrameFactories.newFixedFrameBoundingBox3DBasics(this);

   /**
    * Creates a new empty face.
    *
    * @param initialGuessNormal initial guess for what this face's normal should be. Not modified.
    */
   public FrameFace3D(ReferenceFrameHolder referenceFrameHolder, Vector3DReadOnly initialGuessNormal)
   {
      super(edgeFactory(referenceFrameHolder));
      this.referenceFrameHolder = referenceFrameHolder;
      initialize(initialGuessNormal);
   }

   /**
    * Creates a new empty face.
    *
    * @param initialGuessNormal  initial guess for what this face's normal should be. Not modified.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    */
   public FrameFace3D(ReferenceFrameHolder referenceFrameHolder, Vector3DReadOnly initialGuessNormal, double constructionEpsilon)
   {
      super(edgeFactory(referenceFrameHolder), constructionEpsilon);
      this.referenceFrameHolder = referenceFrameHolder;
      initialize(initialGuessNormal);
   }

   /**
    * Creates a new face given its edges.
    *
    * @param faceEdges           the edges composing the new face. Not modified, reference to the edges
    *                            saved.
    * @param normal              the face's normal. Not modified.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    */
   public FrameFace3D(ReferenceFrameHolder referenceFrameHolder, Collection<FrameHalfEdge3D> faceEdges, FrameVector3DReadOnly normal,
                      double constructionEpsilon)
   {
      super(edgeFactory(referenceFrameHolder), constructionEpsilon);
      this.referenceFrameHolder = referenceFrameHolder;
      initialize(faceEdges, normal);
   }

   private static HalfEdge3DFactory<FrameVertex3D, FrameHalfEdge3D> edgeFactory(ReferenceFrameHolder referenceFrameHolder)
   {
      return (origin, destination) -> new FrameHalfEdge3D(referenceFrameHolder, origin, destination);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrameHolder.getReferenceFrame();
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
   public FixedFrameBoundingBox3DBasics getBoundingBox()
   {
      return boundingBox;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameFace3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameFace3DReadOnly)
         return equals((FrameFace3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this face 3D as follows:
    *
    * <pre>
    * Face 3D: centroid: ( 2.621, -0.723, -1.355 ), normal: ( 0.903, -0.202,  0.378 ), area:  0.180, number of edges: 4
    *    [( 2.590, -0.496, -1.161 ); ( 2.746, -0.536, -1.554 )]
    *    [( 2.746, -0.536, -1.554 ); ( 2.651, -0.950, -1.549 )]
    *    [( 2.651, -0.950, -1.549 ); ( 2.496, -0.910, -1.157 )]
    *    [( 2.496, -0.910, -1.157 ); ( 2.590, -0.496, -1.161 )]
    *    worldFrame
    * </pre>
    *
    * @return the {@code String} representing this face 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameFace3DString(this);
   }
}
