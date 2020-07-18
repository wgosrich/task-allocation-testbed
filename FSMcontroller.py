from statemachine import StateMachine, State

class collisionAvoidance(StateMachine):
    start = State('Start', initial=True)
    straight = State('Straight')
    circle = State('Circle')
    finish = State('Finish')
    # the lines above describe three possible state robots have: start -> straight, straight -> finish, straight -> circle, circle -> ctraight
    begin = start.to(straight)
    encounter = straight.to(circle)
    avoid = circle.to(straight)
    end = straight.to(finish)
    # these are inputs of FSM



