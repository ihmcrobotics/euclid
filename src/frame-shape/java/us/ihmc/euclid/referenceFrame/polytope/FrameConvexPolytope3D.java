package us.ihmc.euclid.referenceFrame.polytope;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DFactory;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DFactory;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DFactory;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a convex polytope 3D expressed in a given reference frame.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameConvexPolytope3D extends AbstractConvexPolytope3D<FrameVertex3D, FrameHalfEdge3D, FrameFace3D>
      implements FrameConvexPolytope3DReadOnly, FrameShape3DBasics, FrameChangeable
{
   /** The reference frame in which this polytope is expressed. */
   private ReferenceFrame referenceFrame;
   /** The centroid of this convex polytope. */
   private final FixedFramePoint3DBasics centroid = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   /** The tightest bounding box entirely containing this face. */
   private final FixedFrameBoundingBox3DBasics boundingBox = EuclidFrameFactories.newFixedFrameBoundingBox3DBasics(this);
   /** Variable to store intermediate results for garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new empty convex polytope initializes its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameConvexPolytope3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new empty convex polytope and initializes its reference frame.
    *
    * @param referenceFrame this polytope initial frame.
    */
   public FrameConvexPolytope3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new empty convex polytope.
    *
    * @param referenceFrame      this polytope initial frame.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public FrameConvexPolytope3D(ReferenceFrame referenceFrame, double constructionEpsilon)
   {
      super(constructionEpsilon);
      setFactories(vertexFactory(this), edgeFactory(this), faceFactory(this));
      initialize();
      setReferenceFrame(referenceFrame);
   }

   /**
    * Creates a new convex polytope and adds vertices provided by the given supplier.
    *
    * @param referenceFrame   this polytope initial frame.
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    */
   public FrameConvexPolytope3D(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier)
   {
      this(referenceFrame, vertex3DSupplier, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new convex polytope, adds vertices provided by the given supplier, its reference frame
    * is initialized to match the reference frame of the vertex supplier.
    *
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    */
   public FrameConvexPolytope3D(FrameVertex3DSupplier vertex3DSupplier)
   {
      this(vertex3DSupplier.getReferenceFrame(), vertex3DSupplier);
   }

   /**
    * Creates a new convex polytope, adds vertices provided by the given supplier.
    *
    * @param referenceFrame      this polytope initial frame.
    * @param vertex3DSupplier    the vertex supplier to get the vertices to add to this convex
    *                            polytope.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public FrameConvexPolytope3D(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier, double constructionEpsilon)
   {
      this(referenceFrame, constructionEpsilon);
      addVertices(vertex3DSupplier);
   }

   /**
    * Creates a new convex polytope, adds vertices provided by the given supplier, its reference frame
    * is initialized to match the reference frame of the vertex supplier.
    *
    * @param vertex3DSupplier    the vertex supplier to get the vertices to add to this convex
    *                            polytope.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public FrameConvexPolytope3D(FrameVertex3DSupplier vertex3DSupplier, double constructionEpsilon)
   {
      this(vertex3DSupplier.getReferenceFrame(), vertex3DSupplier, constructionEpsilon);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param referenceFrame this polytope initial frame.
    * @param other          the other convex polytope to copy. Not modified.
    */
   public FrameConvexPolytope3D(ReferenceFrame referenceFrame, ConvexPolytope3DReadOnly other)
   {
      this(referenceFrame, other.getConstructionEpsilon());
      set(other);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public FrameConvexPolytope3D(FrameConvexPolytope3DReadOnly other)
   {
      this(other.getReferenceFrame(), other);
   }

   private static Vertex3DFactory<FrameVertex3D> vertexFactory(ReferenceFrameHolder referenceFrameHolder)
   {
      return position -> new FrameVertex3D(referenceFrameHolder, position);
   }

   private static HalfEdge3DFactory<FrameVertex3D, FrameHalfEdge3D> edgeFactory(ReferenceFrameHolder referenceFrameHolder)
   {
      return (origin, destination) -> new FrameHalfEdge3D(referenceFrameHolder, origin, destination);
   }

   private static Face3DFactory<FrameFace3D> faceFactory(ReferenceFrameHolder referenceFrameHolder)
   {
      return (initialGuessNormal, constructionEpsilon) -> new FrameFace3D(referenceFrameHolder, initialGuessNormal, constructionEpsilon);
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other polytope to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame {@code this}.
    */
   public void set(FrameConvexPolytope3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      super.set(other);
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other polytope to copy. Not modified.
    */
   public void setIncludingFrame(FrameConvexPolytope3DReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == referenceFrame)
         return;

      // Overwriting the default method so a single transformation is performed.
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getCentroid()
   {
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameBoundingBox3DBasics getBoundingBox()
   {
      return boundingBox;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      return (FrameVertex3DReadOnly) super.getSupportingVertex(supportDirection);
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameShape3DPoseBasics getPose()
   {
      return null;
   }

   /** {@inheritDoc} */
   @Override
   public FrameConvexPolytope3D copy()
   {
      return new FrameConvexPolytope3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameConvexPolytope3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two polytopes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameConvexPolytope3DReadOnly)
         return FrameConvexPolytope3DReadOnly.super.equals((FrameConvexPolytope3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this convex polytope 3D as follows:
    *
    * <pre>
    * Convex polytope 3D: number of: [faces: 4, edges: 12, vertices: 4
    * Face list:
    *    centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    *    centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    *    centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    *    centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list:
    *    [( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list:
    *    ( 0.674,  0.482,  0.712 )
    *    ( 0.870,  0.251,  0.229 )
    *    ( 0.204, -0.803, -0.461 )
    *    (-0.283, -0.207, -0.595 )
    * worldFrame
    * </pre>
    *
    * @return the {@code String} representing this convex polytope 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
