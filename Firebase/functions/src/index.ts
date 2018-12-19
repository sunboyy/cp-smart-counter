import * as functions from 'firebase-functions';
import * as admin from 'firebase-admin';

// // Start writing Firebase Functions
// // https://firebase.google.com/docs/functions/typescript
//
// export const helloWorld = functions.https.onRequest((request, response) => {
//     response.send("Hello from Firebase!");
// });

interface Room {
    count: number;
    max: number;
}

function calculateStatus(value: number, max: number): number {
    if (value < max / 3) return 0;
    if (value > 2 * max / 3) return 2;
    return 1;
}

function statusObject(value: Room) {
    return {
        count: value.count,
        max: value.max,
        status: calculateStatus(value.count, value.max)
    };
}

admin.initializeApp({
    credential: admin.credential.applicationDefault(),
    databaseURL: 'https://cpsmartcounter.firebaseio.com/'
});

const db = admin.database();

export const incrementCount = functions.https.onRequest((request, response) => {
    db.ref('/room/-LTwwzkRWkPHSh2ckiKb').once('value').then(snapshot => {
        const value: Room = snapshot.val();
        value.count++;
        db.ref('/room/-LTwwzkRWkPHSh2ckiKb').set(value).then(() => {
            response.json(statusObject(value));
        }).catch(reason => {
            console.error(reason);
            response.sendStatus(500);
        });
    }).catch(reason => {
        response.sendStatus(500);
    });
});

export const decrementCount = functions.https.onRequest((request, response) => {
    db.ref('/room/-LTwwzkRWkPHSh2ckiKb').once('value').then(snapshot => {
        const value: Room = snapshot.val();
        value.count--;
        if (value.count < 0) value.count = 0;
        db.ref('/room/-LTwwzkRWkPHSh2ckiKb').set(value).then(() => {
            response.json(statusObject(value));
        }).catch(reason => {
            console.error(reason);
            response.sendStatus(500);
        });
    }).catch(reason => {
        console.error(reason);
        response.sendStatus(500);
    });
});

export const goin = functions.https.onRequest((request, response) => {
    const roomId = request.query.room;
    db.ref('/room/' + roomId).once('value').then(snapshot => {
        const value: Room = snapshot.val();
        value.count++;
        db.ref('/room/' + roomId).set(value).then(() => {
            response.json(statusObject(value));
        }).catch(reason => {
            console.error(reason);
            response.sendStatus(500);
        });
    }).catch(reason => {
        response.sendStatus(500);
    });
});

export const goout = functions.https.onRequest((request, response) => {
    const roomId = request.query.room;
    db.ref('/room/' + roomId).once('value').then(snapshot => {
        const value: Room = snapshot.val();
        value.count--;
        if (value.count < 0) value.count = 0;
        db.ref('/room/' + roomId).set(value).then(() => {
            response.json(statusObject(value));
        }).catch(reason => {
            console.error(reason);
            response.sendStatus(500);
        });
    }).catch(reason => {
        console.error(reason);
        response.sendStatus(500);
    });
});

export const move = functions.https.onRequest((request, response) => {
    const fromRoomId = request.query.from;
    const toRoomId = request.query.to;
    db.ref('/room').once('value').then(snapshot => {
        const value = snapshot.val();
        if (value[fromRoomId].count > 0) {
            value[fromRoomId].count--;
            value[toRoomId].count++;
        }
        db.ref('/room').set(value).then(() => {
            response.json(value);
        }).catch(reason => {
            console.error(reason);
            response.sendStatus(500);
        });
    }).catch(reason => {
        console.error(reason);
        response.sendStatus(500);
    });
});

export const resetCount = functions.https.onRequest((request, response) => {
    db.ref('/room').once('value').then(snapshot => {
        const value = snapshot.val();
        Object.keys(value).forEach(key => value[key].count = 0);
        console.log(value);
        db.ref('/room').set(value).then(() => {
            response.sendStatus(200);
        }).catch(reason => {
            console.error(reason);
            response.sendStatus(500);
        });
    }).catch(reason => {
        console.error(reason);
        response.sendStatus(500);
    });
});

export const getStatus = functions.https.onRequest((request, response) => {
    db.ref('/room/-LTwwzkRWkPHSh2ckiKb').once('value').then(snapshot => {
        const value: Room = snapshot.val();
        const result = statusObject(value);
        response.json(result);
    }).catch(reason => {
        console.error(reason);
        response.sendStatus(500);
    });
});

export const room = functions.https.onRequest((request, response) => {
    const roomId: string = request.query.id;
    db.ref('/room/' + roomId).once('value').then(snapshot => {
        const value: Room = snapshot.val();
        const result = statusObject(value);
        response.json(result);
    }).catch(reason => {
        console.error(reason);
        response.sendStatus(500);
    });
});

export const addRoom = functions.https.onRequest((request, response) => {
    const max: number = parseInt(request.query.max);
    db.ref('/room').push({ count: 0, max }).then(snapshot => {
        response.send('Success');
    });
});
