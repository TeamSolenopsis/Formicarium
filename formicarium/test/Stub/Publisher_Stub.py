from formicarium.Interfaces import IPublisher


class Publisher_Stub(IPublisher):
    def __init__(self) -> None:
        self.publisherCalled = False

    def Publish(self, data) -> None:
        self.publisherCalled = True
