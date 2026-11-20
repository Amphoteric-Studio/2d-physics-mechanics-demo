package com.physics.mechanics.demo;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.Contact;
import com.badlogic.gdx.physics.box2d.ContactImpulse;
import com.badlogic.gdx.physics.box2d.ContactListener;
import com.badlogic.gdx.physics.box2d.Fixture;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.Manifold;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.ScreenUtils;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.StretchViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

/**
 * Generic 2D physics mechanics demo using Box2D and procedural ShapeRenderer shapes.
 * Supports desktop keyboard controls and on-screen touch buttons for Android.
 */
public class MechanicsDemo extends ApplicationAdapter {
    // World units (meters) for Box2D simulation
    private static final float WORLD_WIDTH = 24f;
    private static final float WORLD_HEIGHT = 14f;

    // Fixed-timestep simulation parameters
    private static final float TIME_STEP = 1f / 60f;
    private static final int VELOCITY_ITERATIONS = 8;
    private static final int POSITION_ITERATIONS = 3;

    // Falling object configuration
    private static final float SPAWN_INTERVAL = 1.15f;
    private static final float FALLING_OBJECT_LIFETIME = 10f;
    private static final int MAX_FALLING_OBJECTS = 10;

    // Player movement tuning
    private static final float PLAYER_MAX_SPEED = 7.5f;
    private static final float PLAYER_ACCEL = 45f;
    private static final float PLAYER_DECEL = 40f;
    private static final float PLAYER_JUMP_IMPULSE = 12f;

    private enum ShapeKind {
        BOX,
        CIRCLE,
        TRIANGLE
    }

    private static class FallingBody {
        final Body body;
        final ShapeKind kind;
        final float radiusOrHalfSize;
        final Color color;
        float age;

        FallingBody(Body body, ShapeKind kind, float radiusOrHalfSize, Color color) {
            this.body = body;
            this.kind = kind;
            this.radiusOrHalfSize = radiusOrHalfSize;
            this.color = color;
            this.age = 0f;
        }
    }

    private static class TouchButton {
        final float x;
        final float y;
        final float width;
        final float height;
        final String label;
        boolean pressed;

        TouchButton(float x, float y, float width, float height, String label) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.label = label;
        }

        boolean contains(float wx, float wy) {
            return wx >= x && wx <= x + width && wy >= y && wy <= y + height;
        }
    }

    private World world;
    private OrthographicCamera worldCamera;
    private Viewport worldViewport;
    private ShapeRenderer shapeRenderer;

    private Body playerBody;
    private static final float PLAYER_HALF_W = 0.55f;
    private static final float PLAYER_HALF_H = 0.75f;
    private static final Color PLAYER_COLOR = new Color(0.20f, 0.85f, 0.55f, 1f);
    private static final Color GROUND_COLOR = new Color(0.28f, 0.32f, 0.40f, 1f);

    private int groundContacts = 0;
    private float accumulator = 0f;
    private float spawnTimer = 0f;

    private final Array<FallingBody> fallingBodies = new Array<>();
    private final Array<Body> bodiesToDestroy = new Array<>();
    private final Array<TouchButton> touchButtons = new Array<>();

    private final Color[] palette = {
        new Color(0.95f, 0.35f, 0.35f, 1f),
        new Color(0.98f, 0.68f, 0.20f, 1f),
        new Color(0.25f, 0.65f, 0.98f, 1f),
        new Color(0.72f, 0.38f, 0.95f, 1f),
        new Color(0.95f, 0.45f, 0.75f, 1f),
        new Color(0.30f, 0.82f, 0.85f, 1f)
    };

    private TouchButton leftBtn;
    private TouchButton rightBtn;
    private TouchButton jumpBtn;

    @Override
    public void create() {
        worldCamera = new OrthographicCamera();
        worldViewport = new StretchViewport(WORLD_WIDTH, WORLD_HEIGHT, worldCamera);
        worldViewport.apply(true);

        shapeRenderer = new ShapeRenderer();
        world = new World(new Vector2(0f, -14f), true);

        setupContactListener();
        createBoundsAndFloor();
        createPlayer();
        createTouchControls();
    }

    private void setupContactListener() {
        world.setContactListener(new ContactListener() {
            @Override
            public void beginContact(Contact contact) {
                if (isPlayerFloorContact(contact.getFixtureA(), contact.getFixtureB())) {
                    groundContacts++;
                }
            }

            @Override
            public void endContact(Contact contact) {
                if (isPlayerFloorContact(contact.getFixtureA(), contact.getFixtureB())) {
                    groundContacts = Math.max(0, groundContacts - 1);
                }
            }

            @Override
            public void preSolve(Contact contact, Manifold oldManifold) {
            }

            @Override
            public void postSolve(Contact contact, ContactImpulse impulse) {
            }
        });
    }

    private boolean isPlayerFloorContact(Fixture a, Fixture b) {
        Body bodyA = a.getBody();
        Body bodyB = b.getBody();
        boolean aIsPlayer = bodyA == playerBody;
        boolean bIsPlayer = bodyB == playerBody;
        if (!aIsPlayer && !bIsPlayer) return false;

        Object otherData = aIsPlayer ? bodyB.getUserData() : bodyA.getUserData();
        return "floor".equals(otherData) || "falling".equals(otherData);
    }

    private void createBoundsAndFloor() {
        // Solid floor
        BodyDef floorDef = new BodyDef();
        floorDef.type = BodyDef.BodyType.StaticBody;
        floorDef.position.set(WORLD_WIDTH / 2f, 1.0f);

        Body floorBody = world.createBody(floorDef);
        floorBody.setUserData("floor");

        PolygonShape floorShape = new PolygonShape();
        floorShape.setAsBox(WORLD_WIDTH / 2f, 1.0f);

        FixtureDef floorFixture = new FixtureDef();
        floorFixture.shape = floorShape;
        floorFixture.friction = 0.7f;
        floorFixture.restitution = 0.05f;
        floorBody.createFixture(floorFixture);
        floorShape.dispose();

        // Left & right invisible walls to keep player and falling bodies inside view
        createStaticWall(0.25f, WORLD_HEIGHT / 2f, 0.25f, WORLD_HEIGHT / 2f);
        createStaticWall(WORLD_WIDTH - 0.25f, WORLD_HEIGHT / 2f, 0.25f, WORLD_HEIGHT / 2f);
    }

    private void createStaticWall(float x, float y, float halfW, float halfH) {
        BodyDef def = new BodyDef();
        def.type = BodyDef.BodyType.StaticBody;
        def.position.set(x, y);

        Body body = world.createBody(def);
        body.setUserData("wall");

        PolygonShape shape = new PolygonShape();
        shape.setAsBox(halfW, halfH);
        body.createFixture(shape, 0f);
        shape.dispose();
    }

    private void createPlayer() {
        BodyDef def = new BodyDef();
        def.type = BodyDef.BodyType.DynamicBody;
        def.position.set(WORLD_WIDTH / 2f, 3.2f);
        def.fixedRotation = true;

        playerBody = world.createBody(def);
        playerBody.setUserData("player");

        PolygonShape shape = new PolygonShape();
        shape.setAsBox(PLAYER_HALF_W, PLAYER_HALF_H);

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = shape;
        fixtureDef.density = 1.4f;
        fixtureDef.friction = 0.25f;
        fixtureDef.restitution = 0.0f;

        playerBody.createFixture(fixtureDef);
        shape.dispose();
    }

    private void createTouchControls() {
        float btnSize = 2.0f;
        float margin = 0.7f;
        float y = 2.4f;

        leftBtn = new TouchButton(margin, y, btnSize, btnSize, "<");
        rightBtn = new TouchButton(margin + btnSize + 0.45f, y, btnSize, btnSize, ">");
        jumpBtn = new TouchButton(WORLD_WIDTH - margin - btnSize, y, btnSize, btnSize, "^");

        touchButtons.clear();
        touchButtons.add(leftBtn);
        touchButtons.add(rightBtn);
        touchButtons.add(jumpBtn);
    }

    private void updateTouchInput() {
        for (int i = 0; i < touchButtons.size; i++) {
            touchButtons.get(i).pressed = false;
        }

        // Check active pointers (supports multi-touch on Android and mouse on desktop)
        for (int pointer = 0; pointer < 20; pointer++) {
            if (!Gdx.input.isTouched(pointer)) continue;

            Vector2 worldTouch = worldViewport.unproject(new Vector2(Gdx.input.getX(pointer), Gdx.input.getY(pointer)));
            for (int i = 0; i < touchButtons.size; i++) {
                TouchButton btn = touchButtons.get(i);
                if (btn.contains(worldTouch.x, worldTouch.y)) {
                    btn.pressed = true;
                }
            }
        }
    }

    private void handlePlayerMovement(float dt) {
        updateTouchInput();

        float moveDir = 0f;
        boolean moveLeftKey = Gdx.input.isKeyPressed(Input.Keys.A) || Gdx.input.isKeyPressed(Input.Keys.LEFT);
        boolean moveRightKey = Gdx.input.isKeyPressed(Input.Keys.D) || Gdx.input.isKeyPressed(Input.Keys.RIGHT);

        if (moveLeftKey || leftBtn.pressed) moveDir -= 1f;
        if (moveRightKey || rightBtn.pressed) moveDir += 1f;

        Vector2 vel = playerBody.getLinearVelocity();
        float targetVx = moveDir * PLAYER_MAX_SPEED;
        float rate = (Math.abs(moveDir) > 0.01f) ? PLAYER_ACCEL : PLAYER_DECEL;
        float deltaVx = MathUtils.clamp(targetVx - vel.x, -rate * dt, rate * dt);
        float newVx = vel.x + deltaVx;

        boolean jumpJustPressed = Gdx.input.isKeyJustPressed(Input.Keys.W)
            || Gdx.input.isKeyJustPressed(Input.Keys.UP)
            || Gdx.input.isKeyJustPressed(Input.Keys.SPACE);

        float newVy = vel.y;
        if (jumpJustPressed && groundContacts > 0) {
            newVy = PLAYER_JUMP_IMPULSE;
        }

        playerBody.setLinearVelocity(newVx, newVy);
    }

    private void handleTouchJumpEdge() {
        // Trigger jump cleanly on touch down over the jump button
        if (Gdx.input.justTouched()) {
            Vector2 touch = worldViewport.unproject(new Vector2(Gdx.input.getX(), Gdx.input.getY()));
            if (jumpBtn.contains(touch.x, touch.y) && groundContacts > 0) {
                Vector2 vel = playerBody.getLinearVelocity();
                playerBody.setLinearVelocity(vel.x, PLAYER_JUMP_IMPULSE);
            }
        }
    }

    private void spawnFallingObject() {
        if (fallingBodies.size >= MAX_FALLING_OBJECTS) {
            queueDestroy(fallingBodies.first());
            fallingBodies.removeIndex(0);
        }

        float spawnX = MathUtils.random(1.5f, WORLD_WIDTH - 1.5f);
        float spawnY = WORLD_HEIGHT + 0.8f;
        float size = MathUtils.random(0.45f, 0.85f);
        Color color = palette[MathUtils.random(palette.length - 1)];

        ShapeKind[] kinds = ShapeKind.values();
        ShapeKind kind = kinds[MathUtils.random(kinds.length - 1)];

        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyDef.BodyType.DynamicBody;
        bodyDef.position.set(spawnX, spawnY);
        bodyDef.angle = MathUtils.random(0f, MathUtils.PI2);
        bodyDef.angularVelocity = MathUtils.random(-2.2f, 2.2f);

        Body body = world.createBody(bodyDef);
        body.setUserData("falling");

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.density = MathUtils.random(0.8f, 1.8f);
        fixtureDef.friction = 0.45f;
        fixtureDef.restitution = MathUtils.random(0.15f, 0.55f);

        if (kind == ShapeKind.CIRCLE) {
            CircleShape circle = new CircleShape();
            circle.setRadius(size);
            fixtureDef.shape = circle;
            body.createFixture(fixtureDef);
            circle.dispose();
        } else if (kind == ShapeKind.BOX) {
            PolygonShape box = new PolygonShape();
            box.setAsBox(size, size);
            fixtureDef.shape = box;
            body.createFixture(fixtureDef);
            box.dispose();
        } else {
            PolygonShape tri = new PolygonShape();
            Vector2[] vertices = new Vector2[] {
                new Vector2(0f, size * 1.15f),
                new Vector2(-size, -size * 0.85f),
                new Vector2(size, -size * 0.85f)
            };
            tri.set(vertices);
            fixtureDef.shape = tri;
            body.createFixture(fixtureDef);
            tri.dispose();
        }

        fallingBodies.add(new FallingBody(body, kind, size, color));
    }

    private void updateFallingBodies(float dt) {
        spawnTimer += dt;
        if (spawnTimer >= SPAWN_INTERVAL) {
            spawnTimer -= SPAWN_INTERVAL;
            spawnFallingObject();
        }

        for (int i = fallingBodies.size - 1; i >= 0; i--) {
            FallingBody fb = fallingBodies.get(i);
            fb.age += dt;
            if (fb.age >= FALLING_OBJECT_LIFETIME) {
                queueDestroy(fb);
                fallingBodies.removeIndex(i);
            }
        }
    }

    private void queueDestroy(FallingBody fb) {
        if (fb != null && fb.body != null && !bodiesToDestroy.contains(fb.body, true)) {
            bodiesToDestroy.add(fb.body);
        }
    }

    private void flushDestroyedBodies() {
        for (int i = 0; i < bodiesToDestroy.size; i++) {
            Body body = bodiesToDestroy.get(i);
            world.destroyBody(body);
        }
        bodiesToDestroy.clear();
    }

    private void stepPhysics(float dt) {
        float clampedDt = Math.min(dt, 0.05f);
        handlePlayerMovement(clampedDt);
        handleTouchJumpEdge();
        updateFallingBodies(clampedDt);

        accumulator += clampedDt;
        while (accumulator >= TIME_STEP) {
            world.step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
            accumulator -= TIME_STEP;
        }

        flushDestroyedBodies();
    }

    private void renderScene() {
        ScreenUtils.clear(0.10f, 0.12f, 0.16f, 1f);
        worldCamera.update();
        shapeRenderer.setProjectionMatrix(worldCamera.combined);

        // 1. Filled shapes
        shapeRenderer.begin(ShapeRenderer.ShapeType.Filled);

        // Floor
        shapeRenderer.setColor(GROUND_COLOR);
        shapeRenderer.rect(0f, 0f, WORLD_WIDTH, 2.0f);

        // Falling geometric shapes
        for (int i = 0; i < fallingBodies.size; i++) {
            FallingBody fb = fallingBodies.get(i);
            Vector2 pos = fb.body.getPosition();
            float angleDeg = fb.body.getAngle() * MathUtils.radiansToDegrees;

            // Optional subtle fade-out during the last 2 seconds before despawn
            float remaining = FALLING_OBJECT_LIFETIME - fb.age;
            float alpha = remaining < 2f ? MathUtils.clamp(remaining / 2f, 0.15f, 1f) : 1f;

            shapeRenderer.setColor(fb.color.r, fb.color.g, fb.color.b, alpha);

            if (fb.kind == ShapeKind.CIRCLE) {
                shapeRenderer.circle(pos.x, pos.y, fb.radiusOrHalfSize, 24);
            } else if (fb.kind == ShapeKind.BOX) {
                shapeRenderer.rect(
                    pos.x,
                    pos.y,
                    0f,
                    0f,
                    fb.radiusOrHalfSize * 2f,
                    fb.radiusOrHalfSize * 2f,
                    1f,
                    1f,
                    angleDeg
                );
                // Center rectangle properly around origin (pos.x, pos.y)
            }
        }

        // Draw rotated boxes and triangles centered on their body origin
        for (int i = 0; i < fallingBodies.size; i++) {
            FallingBody fb = fallingBodies.get(i);
            if (fb.kind == ShapeKind.CIRCLE) continue;

            Vector2 pos = fb.body.getPosition();
            float angleDeg = fb.body.getAngle() * MathUtils.radiansToDegrees;
            float remaining = FALLING_OBJECT_LIFETIME - fb.age;
            float alpha = remaining < 2f ? MathUtils.clamp(remaining / 2f, 0.15f, 1f) : 1f;
            shapeRenderer.setColor(fb.color.r, fb.color.g, fb.color.b, alpha);

            float s = fb.radiusOrHalfSize;
            if (fb.kind == ShapeKind.BOX) {
                shapeRenderer.rect(pos.x - s, pos.y - s, s, s, s * 2f, s * 2f, 1f, 1f, angleDeg);
            } else if (fb.kind == ShapeKind.TRIANGLE) {
                float cos = MathUtils.cosDeg(angleDeg);
                float sin = MathUtils.sinDeg(angleDeg);

                float lx1 = 0f, ly1 = s * 1.15f;
                float lx2 = -s, ly2 = -s * 0.85f;
                float lx3 = s, ly3 = -s * 0.85f;

                float x1 = pos.x + lx1 * cos - ly1 * sin;
                float y1 = pos.y + lx1 * sin + ly1 * cos;
                float x2 = pos.x + lx2 * cos - ly2 * sin;
                float y2 = pos.y + lx2 * sin + ly2 * cos;
                float x3 = pos.x + lx3 * cos - ly3 * sin;
                float y3 = pos.y + lx3 * sin + ly3 * cos;

                shapeRenderer.triangle(x1, y1, x2, y2, x3, y3);
            }
        }

        // Player body
        Vector2 pPos = playerBody.getPosition();
        shapeRenderer.setColor(PLAYER_COLOR);
        shapeRenderer.rect(
            pPos.x - PLAYER_HALF_W,
            pPos.y - PLAYER_HALF_H,
            PLAYER_HALF_W * 2f,
            PLAYER_HALF_H * 2f
        );

        // On-screen touch controls (useful on Android and testable on desktop)
        boolean showTouchUi = Gdx.app.getType() == Application.ApplicationType.Android || true;
        if (showTouchUi) {
            for (int i = 0; i < touchButtons.size; i++) {
                TouchButton btn = touchButtons.get(i);
                if (btn.pressed) {
                    shapeRenderer.setColor(0.95f, 0.95f, 0.95f, 0.45f);
                } else {
                    shapeRenderer.setColor(1f, 1f, 1f, 0.20f);
                }
                shapeRenderer.rect(btn.x, btn.y, btn.width, btn.height);
            }
        }

        shapeRenderer.end();

        // 2. Crisp directional indicators inside touch buttons (pure ShapeRenderer)
        shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
        shapeRenderer.setColor(1f, 1f, 1f, 0.85f);
        drawButtonGlyph(leftBtn, "<");
        drawButtonGlyph(rightBtn, ">");
        drawButtonGlyph(jumpBtn, "^");
        shapeRenderer.end();
    }

    private void drawButtonGlyph(TouchButton btn, String glyph) {
        float cx = btn.x + btn.width * 0.5f;
        float cy = btn.y + btn.height * 0.5f;
        float d = Math.min(btn.width, btn.height) * 0.24f;

        if ("<".equals(glyph)) {
            shapeRenderer.line(cx + d * 0.5f, cy + d, cx - d * 0.5f, cy);
            shapeRenderer.line(cx - d * 0.5f, cy, cx + d * 0.5f, cy - d);
        } else if (">".equals(glyph)) {
            shapeRenderer.line(cx - d * 0.5f, cy + d, cx + d * 0.5f, cy);
            shapeRenderer.line(cx + d * 0.5f, cy, cx - d * 0.5f, cy - d);
        } else if ("^".equals(glyph)) {
            shapeRenderer.line(cx - d, cy - d * 0.4f, cx, cy + d * 0.6f);
            shapeRenderer.line(cx, cy + d * 0.6f, cx + d, cy - d * 0.4f);
        }
    }

    @Override
    public void render() {
        stepPhysics(Gdx.graphics.getDeltaTime());
        renderScene();
    }

    @Override
    public void resize(int width, int height) {
        if (width <= 0 || height <= 0) return;
        worldViewport.update(width, height, true);
    }

    @Override
    public void dispose() {
        if (shapeRenderer != null) {
            shapeRenderer.dispose();
        }
        if (world != null) {
            world.dispose();
        }
    }
}
